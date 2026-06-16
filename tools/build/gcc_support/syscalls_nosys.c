#include <errno.h>
#include <stddef.h>
#include <sys/stat.h>
#include <sys/types.h>

#undef errno
extern int errno;

int _close(int file)
{
    (void)file;
    errno = EBADF;
    return -1;
}

int _fstat(int file, struct stat *st)
{
    (void)file;
    st->st_mode = S_IFCHR;
    return 0;
}

int _getpid(void)
{
    return 1;
}

int _isatty(int file)
{
    (void)file;
    return 1;
}

int _kill(int pid, int sig)
{
    (void)pid;
    (void)sig;
    errno = EINVAL;
    return -1;
}

off_t _lseek(int file, off_t ptr, int dir)
{
    (void)file;
    (void)ptr;
    (void)dir;
    return 0;
}

ssize_t _read(int file, void *ptr, size_t len)
{
    (void)file;
    (void)ptr;
    (void)len;
    errno = EBADF;
    return -1;
}

void *_sbrk(ptrdiff_t incr)
{
    extern char end;
    extern char _estack;
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &end;
    }

    prev_heap_end = heap_end;
    if (heap_end + incr > &_estack) {
        errno = ENOMEM;
        return (void *)-1;
    }

    heap_end += incr;
    return prev_heap_end;
}

ssize_t _write(int file, const void *ptr, size_t len)
{
    (void)file;
    (void)ptr;
    return (ssize_t)len;
}
