using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Security.Cryptography;

namespace Mono.IO.Ports
{
    internal class LibC
    {
        private const string libc = "libc";

        public const int O_RDWR = 2;
        public const int O_NOCTTY = 0x100;
        public const int O_NONBLOCK = 0x800;

        public const short POLLIN = 0x001;
        public const short POLLOUT = 0x004;

        public const int EINTR = 4;
        public const int EINVAL = 22;
        public const int ENOTTY = 25;


        public const uint CLOCAL = 2048;
        public const uint CREAD = 128;
        public const uint ICANON = 2;
        public const uint ECHO = 8;
        public const uint ECHOE = 16;
        public const uint ECHOK = 32;
        public const uint ECHONL = 64;
        public const uint ISIG = 1;
        public const uint IEXTEN = 32768;
        public const uint OPOST = 1;
        public const uint IGNBRK = 1;

        public const uint CSIZE = 48;
        public const uint CS5 = 0;
        public const uint CS6 = 16;
        public const uint CS7 = 32;
        public const uint CS8 = 48;
        public const uint CSTOPB = 64;

        public const uint INPCK = 16;
        public const uint ISTRIP = 32;

        public const uint PARENB = 256;
        public const uint PARODD = 512;

        public const uint CRTSCTS = 2147483648;
        public const uint IXOFF = 4096;
        public const uint IXON = 1024;

        public const int TCSANOW = 0;
        public const int TCIFLUSH = 0;
        public const int TCOFLUSH = 1;

        public const int FIONREAD = 0x541B;
        public const int TIOCOUTQ = 0x5411;
        public const int TIOCMGET = 0x5415;
        public const int TIOCM_CTS = 0x020;
        public const int TIOCM_CAR = 0x040;
        public const int TIOCM_DTR = 0x002;
        public const int TIOCM_RTS = 0x004;
        public const int TIOCM_DSR = 0x100;
        public const int TIOCMSET = 0x5418;

        public const uint B0 = 0;
        public const uint B50 = 1;
        public const uint B75 = 2;
        public const uint B110 = 3;
        public const uint B134 = 4;
        public const uint B150 = 5;
        public const uint B200 = 6;
        public const uint B300 = 7;
        public const uint B600 = 8;
        public const uint B1200 = 9;
        public const uint B1800 = 10;
        public const uint B2400 = 11;
        public const uint B4800 = 12;
        public const uint B9600 = 13;
        public const uint B19200 = 14;
        public const uint B38400 = 15;
        public const uint B57600 = 4097;
        public const uint B115200 = 4098;
        public const uint B230400 = 4099;
        public const uint B460800 = 4100;
        public const uint B500000 = 4101;
        public const uint B576000 = 4102;
        public const uint B921600 = 4103;
        public const uint B1000000 = 4104;
        public const uint B1152000 = 4105;
        public const uint B1500000 = 4106;
        public const uint B2000000 = 4107;
        public const uint B2500000 = 4108;
        public const uint B3000000 = 4109;
        public const uint B3500000 = 4110;
        public const uint B4000000 = 4111;

        public unsafe struct pollfd
        {
            public int fd;
            public short events;
            public short revents;
        }

        public unsafe struct termios
        {
            private const int NCCS = 32; // note: 16 on powerpc/powerpc64

            public uint c_iflag;
            public uint c_oflag;
            public uint c_cflag;
            public uint c_lflag;
            public byte c_line;
            public fixed byte c_cc[NCCS];
            private uint __c_ispeed;
            private uint __c_ospeed;
        }

        public static unsafe int errno => Marshal.GetLastWin32Error();

        [DllImport(libc, SetLastError = true)]
        public static extern int open(string path, int flags);

        [DllImport(libc, SetLastError = true)]
        public static extern int close(int fd);

        [DllImport(libc, SetLastError = true)]
        public static extern int read(int fd, IntPtr buf, uint count);

        [DllImport(libc, SetLastError = true)]
        public static extern int write(int fd, IntPtr buf, uint count);

        [DllImport(libc, SetLastError = true)]
        public static extern int poll([In, Out] IntPtr fds, uint nfds, int timeout);

        [DllImport(libc, SetLastError = true)]
        public static extern int tcgetattr(int fd, [In, Out] IntPtr p);

        [DllImport(libc, SetLastError = true)]
        public static extern int cfsetospeed(IntPtr p, uint speed);

        [DllImport(libc, SetLastError = true)]
        public static extern int cfsetispeed(IntPtr p, uint speed);

        [DllImport(libc, SetLastError = true)]
        public static extern int tcsetattr(int fd, int optional_actions, IntPtr p);

        [DllImport(libc, SetLastError = true)]
        public static extern int tcflush(int fd, int queue_selector);

        [DllImport(libc, SetLastError = true)]
        public static extern int tcsendbreak(int fd, int duration);

        [DllImport(libc, SetLastError = true)]
        public static extern IntPtr strerror(int errnum);

        [DllImport(libc, SetLastError = true)]
        public static extern int ioctl(int fd, int request, [In, Out] IntPtr arg);
    }

    internal class LibCSerialPortHelper
    {
        public static void ThrowIOException()
        {
            int errnum = Marshal.GetLastWin32Error();
            string error_message = Marshal.PtrToStringAnsi(LibC.strerror(errnum));

            throw new IOException(error_message);
        }

        public static unsafe int read_serial(int fd, Span<byte> buffer, int timeout)
        {
            int error;
            bool poll_result = poll_serial(fd, out error, timeout);

            if (error == -1)
                ThrowIOException();

            if (!poll_result)
                throw new TimeoutException();

            int result = 0;
            var lCount = (uint)buffer.Length;

            fixed (byte* p = buffer)
            {
                result = LibC.read(fd, (IntPtr)p, (uint)buffer.Length);
            }

            if (result == -1)
                ThrowIOException();

            return result;
        }

        public static unsafe int write_serial(int fd, ReadOnlySpan<byte> buffer, int timeout)
        {
            var info = new LibC.pollfd
            {
                fd = fd,
                events = LibC.POLLOUT,
                revents = LibC.POLLOUT,
            };

            uint n = (uint)buffer.Length;
            uint offset = 0;

            while (n > 0)
            {
                int t;

                if (timeout != 0)
                {
                    int c;

                    while ((c = LibC.poll((IntPtr)(&info), 1, timeout)) == -1 && LibC.errno == LibC.EINTR)
                        ;

                    if (c == -1)
                        return -1;
                }

                fixed (byte* p = buffer)
                {
                    do
                    {
                        var pBuff = (IntPtr)(p + offset);
                        t = LibC.write(fd, pBuff, n);
                    } while (t == -1 && LibC.errno == LibC.EINTR);
                }

                if (t < 0)
                    return -1;

                offset += (uint)t;
                n -= (uint)t;
            }

            return 0;
        }

        public static unsafe bool poll_serial(int fd, out int error, int timeout)
        {
            var info = new LibC.pollfd
            {
                fd = fd,
                events = LibC.POLLIN,
                revents = 0,
            };

            error = 0;

            while (LibC.poll((IntPtr)(&info), 1, timeout) == -1 && LibC.errno == LibC.EINTR)
            {
                /* EINTR is an OK condition, we should not throw in the upper layer an IOException */
                if (LibC.errno != LibC.EINTR)
                {
                    error = -1;
                    return false;
                }
            }

            return (info.revents & LibC.POLLIN) != 0;
        }

        public static unsafe bool set_attributes(int fd, int baud_rate, Parity parity, int dataBits, StopBits stopBits, Handshake handshake)
        {
            LibC.termios newtio = new LibC.termios();

            if (LibC.tcgetattr(fd, (IntPtr)(&newtio)) == -1)
                return false;

            newtio.c_cflag |= (LibC.CLOCAL | LibC.CREAD);
            newtio.c_lflag &= ~(LibC.ICANON | LibC.ECHO | LibC.ECHOE | LibC.ECHOK | LibC.ECHONL | LibC.ISIG | LibC.IEXTEN);
            newtio.c_oflag &= ~(LibC.OPOST);
            newtio.c_iflag = LibC.IGNBRK;

            baud_rate = (int)setup_baud_rate(baud_rate, out var custom_baud_rate);

            /* char lenght */
            newtio.c_cflag &= ~LibC.CSIZE;

            switch (dataBits)
            {
                case 5:
                    newtio.c_cflag |= LibC.CS5;
                    break;
                case 6:
                    newtio.c_cflag |= LibC.CS6;
                    break;
                case 7:
                    newtio.c_cflag |= LibC.CS7;
                    break;
                case 8:
                default:
                    newtio.c_cflag |= LibC.CS8;
                    break;
            }

            switch (stopBits)
            {
                case StopBits.None:
                    /* Unhandled */
                    break;
                case StopBits.One: /* One */
                    /* do nothing, the default is one stop bit */
                    newtio.c_cflag &= ~LibC.CSTOPB;
                    break;
                case StopBits.Two: /* Two */
                    newtio.c_cflag |= LibC.CSTOPB;
                    break;
                case StopBits.OnePointFive: /* OnePointFive */
                    /* 8250 UART handles stop bit flag as 1.5 stop bits for 5 data bits */
                    newtio.c_cflag |= LibC.CSTOPB;
                    break;
            }

            /* parity */
            newtio.c_iflag &= ~(LibC.INPCK | LibC.ISTRIP);

            switch (parity)
            {
                case Parity.None: /* None */
                    newtio.c_cflag &= ~(LibC.PARENB | LibC.PARODD);
                    break;

                case Parity.Odd: /* Odd */
                    newtio.c_cflag |= LibC.PARENB | LibC.PARODD;
                    break;

                case Parity.Even: /* Even */
                    newtio.c_cflag &= ~(LibC.PARODD);
                    newtio.c_cflag |= (LibC.PARENB);
                    break;

                case Parity.Mark: /* Mark */
                    /* XXX unhandled */
                    break;
                case Parity.Space: /* Space */
                    /* XXX unhandled */
                    break;
            }

            newtio.c_cflag &= ~LibC.CRTSCTS;

            switch (handshake)
            {
                case Handshake.None: /* None */
                    /* do nothing */
                    break;
                case Handshake.RequestToSend: /* RequestToSend (RTS) */
                    newtio.c_cflag |= LibC.CRTSCTS;
                    break;
                case Handshake.RequestToSendXOnXOff: /* RequestToSendXOnXOff (RTS + XON/XOFF) */
                    newtio.c_cflag |= LibC.CRTSCTS;
                    newtio.c_iflag |= LibC.IXOFF | LibC.IXON;
                    break;
                case Handshake.XOnXOff: /* XOnXOff */
                    newtio.c_iflag |= LibC.IXOFF | LibC.IXON;
                    break;
            }

            var ptrNewtio = (IntPtr)(&newtio);

            if (!custom_baud_rate)
            {
                if (LibC.cfsetospeed(ptrNewtio, (uint)baud_rate) < 0 || LibC.cfsetispeed(ptrNewtio, (uint)baud_rate) < 0)
                    return false;
            }
            else
            {
                if (LibC.cfsetospeed(ptrNewtio, LibC.B38400) < 0 || LibC.cfsetispeed(ptrNewtio, LibC.B38400) < 0)
                    return false;
            }

            if (LibC.tcsetattr(fd, LibC.TCSANOW, ptrNewtio) < 0)
                return false;

            //            if (custom_baud_rate  TRUE)
            //            {
            //#if defined(HAVE_LINUX_SERIAL_H)
            //		struct serial_struct ser;

            //		if (ioctl (fd, TIOCGSERIAL, &ser) < 0)
            //		{
            //			return FALSE;
            //		}

            //		ser.custom_divisor = ser.baud_base / baud_rate;
            //		ser.flags &= ~ASYNC_SPD_MASK;
            //		ser.flags |= ASYNC_SPD_CUST;

            //		if (ioctl (fd, TIOCSSERIAL, &ser) < 0)
            //		{
            //			return FALSE;
            //		}
            //#elif defined(HAVE_IOKIT)
            //		speed_t speed = baud_rate;
            //		if (ioctl(fd, IOSSIOSPEED, &speed) == -1)
            //			return FALSE;
            //#else
            //                /* Don't know how to set custom baud rate on this platform. */
            //                return FALSE;
            //#endif
            //            }

            return true;
        }

        private static uint setup_baud_rate(int baud_rate, out bool custom_baud_rate)
        {
            custom_baud_rate = false;

            switch (baud_rate)
            {
                case 921600: return LibC.B921600;
                case 460800: return LibC.B460800;
                case 230400: return LibC.B230400;
                case 115200: return LibC.B115200;
                case 57600: return LibC.B57600;
                case 38400: return LibC.B38400;
                case 19200: return LibC.B19200;
                case 9600: return LibC.B9600;
                case 4800: return LibC.B4800;
                case 2400: return LibC.B2400;
                case 1800: return LibC.B1800;
                case 1200: return LibC.B1200;
                case 600: return LibC.B600;
                case 300: return LibC.B300;
                case 200: return LibC.B200;
                case 150: return LibC.B150;
                case 134: return LibC.B134;
                case 110: return LibC.B110;
                case 75: return LibC.B75;
                case 50: return LibC.B50;
                case 0: return LibC.B0;
                default: custom_baud_rate = true; break;
            }

            return (uint)baud_rate;
        }

        public static unsafe int get_bytes_in_buffer(int fd, bool input)
        {
            int retval;
            return LibC.ioctl(fd, input ? LibC.FIONREAD : LibC.TIOCOUTQ, (IntPtr)(&retval)) == -1 ? -1 : retval;
        }

        public static unsafe SerialSignal get_signals(int fd, out bool error)
        {
            int signals;
            error = false;

            if (LibC.ioctl(fd, LibC.TIOCMGET, (IntPtr)(&signals)) == -1)
            {
                error = true;
                return SerialSignal.None;
            }

            return get_signal_codes(signals);
        }

        private static SerialSignal get_signal_codes(int signals)
        {
            SerialSignal retval = SerialSignal.None;

            if ((signals & LibC.TIOCM_CAR) != 0)
                retval |= SerialSignal.Cd;

            if ((signals & LibC.TIOCM_CTS) != 0)
                retval |= SerialSignal.Cts;

            if ((signals & LibC.TIOCM_DSR) != 0)
                retval |= SerialSignal.Dsr;

            if ((signals & LibC.TIOCM_DTR) != 0)
                retval |= SerialSignal.Dtr;

            if ((signals & LibC.TIOCM_RTS) != 0)
                retval |= SerialSignal.Rts;

            return retval;
        }

        private static int get_signal_code(SerialSignal signal)
        {
            switch (signal)
            {
                case SerialSignal.Cd:
                    return LibC.TIOCM_CAR;
                case SerialSignal.Cts:
                    return LibC.TIOCM_CTS;
                case SerialSignal.Dsr:
                    return LibC.TIOCM_DSR;
                case SerialSignal.Dtr:
                    return LibC.TIOCM_DTR;
                case SerialSignal.Rts:
                    return LibC.TIOCM_RTS;
                default:
                    return 0;
            }
        }

        public static unsafe int set_signal(int fd, SerialSignal signal, bool value)
        {
            int signals, expected;
            expected = get_signal_code(signal);

            if (LibC.ioctl(fd, LibC.TIOCMGET, (IntPtr)(&signals)) == -1)
            {
                /* Return successfully for pseudo-ttys.
                 * Linux kernels < 5.13 return EINVAL,
                 * but versions >=5.13 return ENOTTY. */
                if (LibC.errno == LibC.EINVAL || LibC.errno == LibC.ENOTTY)
                    return 1;

                return -1;
            }

            bool activated = (signals & expected) != 0;

            if (activated == value) /* Already set */
                return 1;

            if (value)
                signals |= expected;
            else
                signals &= ~expected;

            if (LibC.ioctl(fd, LibC.TIOCMSET, (IntPtr)(&signals)) == -1)
                return -1;

            return 1;
        }
    }
}
