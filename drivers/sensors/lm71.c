/****************************************************************************
 * drivers/sensors/lm71.c
 * Character driver for the TI LM71 Temperature Sensor
 *
 *   Copyright (C) 2019, Augusto Fraga Giachero. All rights reserved.
 *   Copyright (C) 2011, 2013, 2016 Gregory Nutt. All rights reserved.
 *   Author: Augusto Fraga Giachero <afg@augustofg.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdlib.h>
#include <fixedmath.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/fs/fs.h>
#include <nuttx/spi/spi.h>
#include <nuttx/sensors/lm71.h>
#include <nuttx/random.h>

#if defined(CONFIG_SPI) && defined(CONFIG_SENSORS_LM71)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LM71_SPI_FREQUENCY
#  define CONFIG_LM71_SPI_FREQUENCY 1000000
#endif

#define LM71_SPI_MODE (SPIDEV_MODE0) /* SPI Mode 0: CPOL=0, CPHA=0 */

/* Centigrade to Fahrenheit conversion:  F = 9*C/5 + 32 */

#define B16_9DIV5  (9 * 65536 / 5)
#define B16_32     (32 * 65536)

#define LM71_ID 0x800F

/****************************************************************************
 * Private
 ****************************************************************************/

struct lm71_dev_s
{
  FAR struct spi_dev_s *spi;    /* Saved SPI driver instance */
  int spidev;
  bool fahrenheit;              /* true: temperature will be reported in fahrenheit */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
/* SPI Helpers */

static int lm71_readtemp(FAR struct lm71_dev_s *priv, FAR b16_t *temp);

static uint16_t lm71_shutdown(FAR struct lm71_dev_s *priv, int shutdown);

/* Character driver methods */

static int lm71_open(FAR struct file *filep);
static int lm71_close(FAR struct file *filep);
static ssize_t lm71_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t lm71_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int lm71_ioctl(FAR struct file *filep,int cmd,unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_lm71fops =
{
  lm71_open,
  lm71_close,
  lm71_read,
  lm71_write,
  NULL,
  lm71_ioctl,
  NULL
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void lm71_configspi(FAR struct spi_dev_s *spi)
{
  /* Configure SPI for the LM71 */

  SPI_SETMODE(spi, LM71_SPI_MODE);
  SPI_SETBITS(spi, 8);
  SPI_HWFEATURES(spi, 0);
  SPI_SETFREQUENCY(spi, CONFIG_LM71_SPI_FREQUENCY);
}

/****************************************************************************
 * Name: lm71_shutdown
 *
 * Description:
 *   If shutdown is = 0, put the sensor in continuous measurement,
 *   mode. Otherwise put in shutdown mode. Returns the manufacturer's
 *   device ID.
 *
 ****************************************************************************/

static uint16_t lm71_shutdown(FAR struct lm71_dev_s *priv, int shutdown)
{
  uint8_t buf[2];
  uint8_t mode = shutdown ? 0xFF : 0x00;

  SPI_LOCK(priv->spi, true);
  lm71_configspi(priv->spi);
  SPI_SELECT(priv->spi, priv->spidev, true);

  SPI_RECVBLOCK(priv->spi, buf, 2);
  SPI_SEND(priv->spi, mode);
  SPI_SEND(priv->spi, mode);
  SPI_RECVBLOCK(priv->spi, buf, 2);

  SPI_SELECT(priv->spi, priv->spidev, false);
  SPI_LOCK(priv->spi, false);

  return (buf[0] << 8) | buf[1];
}

/****************************************************************************
 * Name: lm71_readtemp
 *
 * Description:
 *   Read the temperature register with special scaling
 *
 ****************************************************************************/

static int lm71_readtemp(FAR struct lm71_dev_s *priv, FAR b16_t *temp)
{
  uint8_t buf[2];
  int16_t temp_raw;
  b16_t temp16;

  /* Read the raw temperature data */

  SPI_LOCK(priv->spi, true);
  lm71_configspi(priv->spi);
  SPI_SELECT(priv->spi, priv->spidev, true);

  SPI_RECVBLOCK(priv->spi, buf, 2);

  SPI_SELECT(priv->spi, priv->spidev, false);
  SPI_LOCK(priv->spi, false);

  temp_raw = (int16_t) (((buf[0] << 8) | buf[1]) & 0xFFFC);

  /* Convert from 9.7 bits to 16.16 bits */

  temp16 = (int32_t)temp_raw << 9;

  add_sensor_randomness(temp16);

  sninfo("Centigrade: %08x\n", temp16);

  /* Was fahrenheit requested? */

  if (priv->fahrenheit)
    {
      /* Centigrade to Fahrenheit conversion:  F = 9*C/5 + 32 */

      temp16 =  b16mulb16(temp16, B16_9DIV5) + B16_32;
      sninfo("Fahrenheit: %08x\n", temp16);
    }

  *temp = temp16;
  return OK;
}

/****************************************************************************
 * Name: lm71_open
 *
 * Description:
 *   This function is called whenever the LM71 device is opened.
 *
 ****************************************************************************/

static int lm71_open(FAR struct file *filep)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lm71_dev_s *priv = inode->i_private;
  uint16_t id;

  id = lm71_shutdown(priv, 1);
  lm71_shutdown(priv, 0);
  if (id != LM71_ID)
    {
      set_errno(ENODEV);
      return -ENODEV;
    }
  return OK;
}

/****************************************************************************
 * Name: lm71_close
 *
 * Description:
 *   This routine is called when the LM71 device is closed.
 *
 ****************************************************************************/

static int lm71_close(FAR struct file *filep)
{
  return OK;
}

/****************************************************************************
 * Name: lm71_read
 ****************************************************************************/

static ssize_t lm71_read(FAR struct file *filep, FAR char *buffer, size_t buflen)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lm71_dev_s *priv = inode->i_private;
  FAR b16_t *ptr;
  ssize_t nsamples;
  int i;
  int ret;

  /* How many samples were requested to get? */

  nsamples = buflen / sizeof(b16_t);
  ptr      = (FAR b16_t *)buffer;

  sninfo("buflen: %d nsamples: %d\n", buflen, nsamples);

  /* Get the requested number of samples */

  for (i = 0; i < nsamples; i++)
    {
      b16_t temp = 0;

      /* Read the next b16_t temperature value */

      ret = lm71_readtemp(priv, &temp);
      if (ret < 0)
        {
          snerr("ERROR: lm71_readtemp failed: %d\n", ret);
          return (ssize_t)ret;
        }

      /* Save the temperature value in the user buffer */

      *ptr++ = temp;
    }

  return nsamples * sizeof(b16_t);
}

/****************************************************************************
 * Name: lm71_write
 ****************************************************************************/

static ssize_t lm71_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: lm71_ioctl
 ****************************************************************************/

static int lm71_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct lm71_dev_s *priv = inode->i_private;
  int ret = OK;

  switch (cmd)
    {

      case SNIOC_SHUTDOWN:
        lm71_shutdown(priv, 1);
        sninfo("Sensor shutdown\n");
        break;

      case SNIOC_POWERUP:
        lm71_shutdown(priv, 0);
        sninfo("Sensor powerup\n");
        break;

      /* Report samples in Fahrenheit */
      case SNIOC_FAHRENHEIT:
        priv->fahrenheit = true;
        sninfo("Fahrenheit\n");
        break;

      /* Report Samples in Centigrade */

      case SNIOC_CENTIGRADE:
        priv->fahrenheit = false;
        sninfo("Centigrade\n");
        break;

      default:
        sninfo("Unrecognized cmd: %d\n", cmd);
        ret = -ENOTTY;
        break;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lm71_register
 *
 * Description:
 *   Register the LM71 character device as 'devpath'
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/temp0"
 *   spi - An instance of the SPI interface to use to communicate with LM71
 *   spidev - The SPI device number used to select the correct CS line
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lm71_register(FAR const char *devpath, FAR struct spi_dev_s *spi, int spidev)
{
  FAR struct lm71_dev_s *priv;
  int ret;

  /* Sanity check */

  DEBUGASSERT(spi != NULL);

  /* Initialize the LM71 device structure */

  priv = (FAR struct lm71_dev_s *)kmm_malloc(sizeof(struct lm71_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate instance\n");
      return -ENOMEM;
    }

  priv->spi        = spi;
  priv->spidev     = spidev;
  priv->fahrenheit = false;

  /* Register the character driver */

  ret = register_driver(devpath, &g_lm71fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
#endif /* CONFIG_SPI && CONFIG_SENSORS_LM71 */
