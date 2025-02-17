#include <Printf.h>

extern "C"
{
    ssize_t _write(int file, const char *ptr, ssize_t len);
}

int _write(int file, const char *ptr, ssize_t len)
{
    int i;

    if (file == STDOUT_FILENO || file == STDERR_FILENO)
    {
        for (i = 0; i < len; i++)
        {
            if (ptr[i] == '\n')
            {
                usart_send_blocking(USART1, '\r');
            }
            usart_send_blocking(USART1, ptr[i]);
        }
        return i;
    }
    errno = EIO;
    return -1;
}
