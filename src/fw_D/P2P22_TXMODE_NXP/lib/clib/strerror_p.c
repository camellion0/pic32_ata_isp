/*                      - STRERROR_P.C -

   The ANSI "strerror" function.

   Version: 1.03 [IANP]

*/

#include "errno.h"
#include "pgmspace.h"

#pragma language=extended

/* Provide your own error messages here (see errno.h): */
static __flash char EZERO_msg[] = "EZERO";
static __flash char EPERM_msg[] = "EPERM";
static __flash char ENOENT_msg[] = "ENOENT";
static __flash char ESRCH_msg[] = "ESRCH";
static __flash char EINTR_msg[] = "EINTR";
static __flash char EIO_msg[] = "EIO";
static __flash char ENXIO_msg[] = "ENXIO";
static __flash char E2BIG_msg[] = "E2BIG";
static __flash char ENOEXEC_msg[] = "ENOEXEC";
static __flash char EBADF_msg[] = "EBADF";
static __flash char ECHILD_msg[] = "ECHILD";
static __flash char EAGAIN_msg[] = "EAGAIN";
static __flash char ENOMEM_msg[] = "ENOMEM";
static __flash char EACCES_msg[] = "EACCES";
static __flash char EFAULT_msg[] = "EFAULT";
static __flash char ENOTBLK_msg[] = "ENOTBLK";
static __flash char EBUSY_msg[] = "EBUSY";
static __flash char EEXIST_msg[] = "EEXIST";
static __flash char EXDEV_msg[] = "EXDEV";
static __flash char ENODEV_msg[] = "ENODEV";
static __flash char ENOTDIR_msg[] = "ENOTDIR";
static __flash char EISDIR_msg[] = "EISDIR";
static __flash char EINVAL_msg[] = "EINVAL";
static __flash char ENFILE_msg[] = "ENFILE";
static __flash char EMFILE_msg[] = "EMFILE";
static __flash char ENOTTY_msg[] = "ENOTTY";
static __flash char ETXTBSY_msg[] = "ETXTBSY";
static __flash char EFBIG_msg[] = "EFBIG";
static __flash char ENOSPC_msg[] = "ENOSPC";
static __flash char ESPIPE_msg[] = "ESPIPE";
static __flash char EROFS_msg[] = "EROFS";
static __flash char EMLINK_msg[] = "EMLINK";
static __flash char EPIPE_msg[] = "EPIPE";
static __flash char EDOM_msg[] = "EDOM";
static __flash char ERANGE_msg[] = "ERANGE";
static __flash char EUCLEAN_msg[] = "EUCLEAN";
static __flash char EDEADLOCK_msg[] = "EDEADLOCK";
static __flash char EILSEQ_msg[] = "EILSEQ";
static __flash char unknown_error_msg[] = "(unknown error)";

PGM_P strerror_P(int errnum)
  {
    /* errors defined in errno.h */
    switch(errnum)
      {
        case EZERO:
          return EZERO_msg;
        case EPERM:
          return EPERM_msg;
        case ENOENT:
          return ENOENT_msg;
        case ESRCH:
          return ESRCH_msg;
        case EINTR:
          return EINTR_msg;
        case EIO:
          return EIO_msg;
        case ENXIO:
          return ENXIO_msg;
        case E2BIG:
          return E2BIG_msg;
        case ENOEXEC:
          return ENOEXEC_msg;
        case EBADF:
          return EBADF_msg;
        case ECHILD:
          return ECHILD_msg;
        case EAGAIN:
          return EAGAIN_msg;
        case ENOMEM:
          return ENOMEM_msg;
        case EACCES:
          return EACCES_msg;
        case EFAULT:
          return EFAULT_msg;
        case ENOTBLK:
          return ENOTBLK_msg;
        case EBUSY:
          return EBUSY_msg;
        case EEXIST:
          return EEXIST_msg;
        case EXDEV:
          return EXDEV_msg;
        case ENODEV:
          return ENODEV_msg;
        case ENOTDIR:
          return ENOTDIR_msg;
        case EISDIR:
          return EISDIR_msg;
        case EINVAL:
          return EINVAL_msg;
        case ENFILE:
          return ENFILE_msg;
        case EMFILE:
          return EMFILE_msg;
        case ENOTTY:
          return ENOTTY_msg;
        case ETXTBSY:
          return ETXTBSY_msg;
        case EFBIG:
          return EFBIG_msg;
        case ENOSPC:
          return ENOSPC_msg;
        case ESPIPE:
          return ESPIPE_msg;
        case EROFS:
          return EROFS_msg;
        case EMLINK:
          return EMLINK_msg;
        case EPIPE:
          return EPIPE_msg;
        case EDOM:
          return EDOM_msg;
        case ERANGE:
          return ERANGE_msg;
        case EUCLEAN:
          return EUCLEAN_msg;
        case EDEADLOCK:
          return EDEADLOCK_msg;
        case EILSEQ:
          return EILSEQ_msg;
        default:
          return unknown_error_msg;
      }
  }
