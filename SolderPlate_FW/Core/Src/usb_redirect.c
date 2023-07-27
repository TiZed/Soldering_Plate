
#include <errno.h>
#include <stdio.h>
#include  <sys/stat.h>
#include  <sys/unistd.h>

#include "usbd_cdc_if.h"
#include "usb_redirect.h"

int _write(int fd, char* ptr, int len) {
  USBD_StatusTypeDef hstatus;

  if (fd == STDOUT_FILENO || fd == STDERR_FILENO) {
    hstatus = CDC_Transmit_FS((uint8_t *) ptr, len) ; 
    if (hstatus == USBD_OK)
      return len;
    else
      return EIO;
  }
  errno = EBADF;
  return -1;
}
