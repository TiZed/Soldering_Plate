
#include <errno.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#include "usbd_cdc_if.h"
#include "usb_redirect.h"


int _write(int fd, char* ptr, int len) {
  USBD_StatusTypeDef hstatus = USBD_BUSY ;

  if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
    while(hstatus == USBD_BUSY) {
      hstatus = CDC_Transmit_FS((uint8_t *) ptr, len) ;
    }
    if (hstatus == USBD_OK) return len ;
    else
    {
      errno = EIO;
      return -1;
    }
  }
  errno = EBADF;
  return -1;
}

int __io_putchar(int ch)
{
    USBD_StatusTypeDef hstatus = USBD_BUSY ;
    while(hstatus == USBD_BUSY) {
      hstatus = CDC_Transmit_FS((uint8_t *) &ch, 1) ;
    }
    if (hstatus == USBD_OK) return 1 ;
    return 0;
}