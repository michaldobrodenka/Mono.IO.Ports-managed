//
// System.IO.Ports.SerialPortStream.cs
//
// Authors:
//	Chris Toshok (toshok@ximian.com)
//	Carlos Alberto Cortez (calberto.cortez@gmail.com)
//
// (c) Copyright 2006 Novell, Inc. (http://www.novell.com)
//
// Slightly modified by Konrad M. Kruczynski (added baud rate value checking)


using System;
using System.Drawing;
using System.IO;
using System.Runtime.InteropServices;

namespace Mono.IO.Ports
{
    class SerialPortStream : Stream, ISerialStream, IDisposable
    {
        int fd;
        int read_timeout;
        int write_timeout;
        bool disposed;

        public SerialPortStream(string portName, int baudRate, int dataBits, Parity parity, StopBits stopBits,
                bool dtrEnable, bool rtsEnable, Handshake handshake, int readTimeout, int writeTimeout,
                int readBufferSize, int writeBufferSize)
        {
            fd = LibC.open(portName, LibC.O_RDWR | LibC.O_NONBLOCK | LibC.O_NOCTTY);

            if (fd == -1)
                LibCSerialPortHelper.ThrowIOException();

            if (!LibCSerialPortHelper.set_attributes(fd, baudRate, parity, dataBits, stopBits, handshake))
                LibCSerialPortHelper.ThrowIOException(); // Probably Win32Exc for compatibility

            read_timeout = readTimeout;
            write_timeout = writeTimeout;

            SetSignal(SerialSignal.Dtr, dtrEnable);

            if (handshake != Handshake.RequestToSend && handshake != Handshake.RequestToSendXOnXOff)
                SetSignal(SerialSignal.Rts, rtsEnable);
        }

        public override bool CanRead
        {
            get
            {
                return true;
            }
        }

        public override bool CanSeek
        {
            get
            {
                return false;
            }
        }

        public override bool CanWrite
        {
            get
            {
                return true;
            }
        }

        public override bool CanTimeout
        {
            get
            {
                return true;
            }
        }

        public override int ReadTimeout
        {
            get
            {
                return read_timeout;
            }
            set
            {
                if (value < 0 && value != SerialPort.InfiniteTimeout)
                    throw new ArgumentOutOfRangeException("value");

                read_timeout = value;
            }
        }

        public override int WriteTimeout
        {
            get
            {
                return write_timeout;
            }
            set
            {
                if (value < 0 && value != SerialPort.InfiniteTimeout)
                    throw new ArgumentOutOfRangeException("value");

                write_timeout = value;
            }
        }

        public override long Length
        {
            get
            {
                throw new NotSupportedException();
            }
        }

        public override long Position
        {
            get
            {
                throw new NotSupportedException();
            }
            set
            {
                throw new NotSupportedException();
            }
        }

        public override void Flush()
        {
            // If used, this _could_ flush the serial port
            // buffer (not the SerialPort class buffer)
        }

        public unsafe override int Read([In, Out] byte[] buffer, int offset, int count)
        {
            return this.Read(buffer.AsSpan(offset, count));
        }

        public unsafe override int Read(Span<byte> buffer)
        {
            CheckDisposed();

            if (buffer == null)
                throw new ArgumentNullException("buffer");

            if (buffer.Length == 0)
                return 0;

            if (buffer.Length < 0)
                throw new ArgumentOutOfRangeException("offset or count less than zero.");

            return LibCSerialPortHelper.read_serial(fd, buffer, read_timeout);
        }

        public override long Seek(long offset, SeekOrigin origin)
        {
            throw new NotSupportedException();
        }

        public override void SetLength(long value)
        {
            throw new NotSupportedException();
        }

        public override void Write(byte[] buffer, int offset, int count)
        {
            this.Write(new ReadOnlySpan<byte>(buffer, offset, count));
        }

        public override void Write(ReadOnlySpan<byte> buffer)
        {
            CheckDisposed();

            if (buffer == null)
                throw new ArgumentNullException("buffer");

            if (buffer.Length < 0)
                throw new ArgumentOutOfRangeException();

            if (buffer.Length == 0)
                return;

            // FIXME: this reports every write error as timeout
            if (LibCSerialPortHelper.write_serial(fd, buffer, write_timeout) < 0)
                throw new TimeoutException("The operation has timed-out");
        }

        protected override void Dispose(bool disposing)
        {
            if (disposed)
                return;

            disposed = true;

            if (LibC.close(fd) != 0)
                LibCSerialPortHelper.ThrowIOException();
        }

        public override void Close()
        {
            ((IDisposable)this).Dispose();
        }

        void IDisposable.Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        ~SerialPortStream()
        {
            try
            {
                Dispose(false);
            }
            catch (IOException)
            {
            }
        }

        void CheckDisposed()
        {
            if (disposed)
                throw new ObjectDisposedException(GetType().FullName);
        }

        public void SetAttributes(int baud_rate, Parity parity, int data_bits, StopBits sb, Handshake hs)
        {
            if (!LibCSerialPortHelper.set_attributes(fd, baud_rate, parity, data_bits, sb, hs))
                LibCSerialPortHelper.ThrowIOException();
        }

        public int BytesToRead
        {
            get
            {
                int result = LibCSerialPortHelper.get_bytes_in_buffer(fd, true);

                if (result == -1)
                    LibCSerialPortHelper.ThrowIOException();

                return result;
            }
        }

        public int BytesToWrite
        {
            get
            {
                int result = LibCSerialPortHelper.get_bytes_in_buffer(fd, false);

                if (result == -1)
                    LibCSerialPortHelper.ThrowIOException();

                return result;
            }
        }

        public void DiscardInBuffer()
        {
            if (LibC.tcflush(fd, LibC.TCIFLUSH) != 0)
                LibCSerialPortHelper.ThrowIOException();
        }

        public void DiscardOutBuffer()
        {
            if (LibC.tcflush(fd, LibC.TCOFLUSH) != 0)
                LibCSerialPortHelper.ThrowIOException();
        }

        public SerialSignal GetSignals()
        {
            SerialSignal signals = LibCSerialPortHelper.get_signals(fd, out var error);

            if (error)
                LibCSerialPortHelper.ThrowIOException();

            return signals;
        }

        public void SetSignal(SerialSignal signal, bool value)
        {
            if (signal < SerialSignal.Cd || signal > SerialSignal.Rts || signal == SerialSignal.Cd || signal == SerialSignal.Cts || signal == SerialSignal.Dsr)
                throw new Exception("Invalid internal value");

            if (LibCSerialPortHelper.set_signal(fd, signal, value) == -1)
                LibCSerialPortHelper.ThrowIOException();
        }

        public void SetBreakState(bool value)
        {
            if (value && LibC.tcsendbreak(fd, 0) == -1)
                LibCSerialPortHelper.ThrowIOException();
        }
    }
}


