using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace Mono.IO.Ports
{
	class WindowsComPortsEnumerator
	{
		[DllImport("advapi32.dll", EntryPoint = "RegOpenKeyExW", CharSet = CharSet.Unicode, BestFitMapping = false)]
		private static extern int RegOpenKeyEx(
			IntPtr hKey,
			string lpSubKey,
			int ulOptions,
			int samDesired,
			out IntPtr hkResult);

		[DllImport("advapi32.dll")]
		private static extern int RegCloseKey(IntPtr hKey);

		[DllImport("advapi32.dll", EntryPoint = "RegEnumValueW", CharSet = CharSet.Unicode, BestFitMapping = false)]
		private static extern int RegEnumValue(
			IntPtr hKey,
			int dwIndex,
			char[] lpValueName,
			ref int lpcbValueName,
			IntPtr lpReserved_MustBeZero,
			int[]? lpType,
			byte[]? lpData,
			int[]? lpcbData);

		[DllImport("advapi32.dll", EntryPoint = "RegQueryValueExW", CharSet = CharSet.Unicode, BestFitMapping = false)]
		private static extern int RegQueryValueEx(
			IntPtr hKey,
			string lpValueName,
			int[]? lpReserved,
			ref int lpType,
			[Out] char[] lpData,
			ref int lpcbData);

		public static List<string> GetPorts()
		{
			var HKEY_LOCAL_MACHINE = new IntPtr(-2147483646);
			const int KEY_ENUMERATE_SUB_KEYS = 8;
			const int KEY_READ = 0x20019;
			const int REG_SZ = 1;

			var r = new List<string>();

			try
			{
				// if there are no serial ports at all, the registry key is not there at all
				var errorCode = RegOpenKeyEx(HKEY_LOCAL_MACHINE, @"HARDWARE\DEVICEMAP\SERIALCOMM", 0, KEY_ENUMERATE_SUB_KEYS | KEY_READ, out var serialCommHkey);
				if (errorCode != 0)
					return r;

				try
				{
					var chArray = new char[300];

					for (;;)
					{
						var len = chArray.Length;
						errorCode = RegEnumValue(serialCommHkey, r.Count, chArray, ref len, IntPtr.Zero, null, null, null);
						if (errorCode != 0)
							break;

						var keyName = new string(chArray, 0, len);

						var lpType = 0;
						var dataLen = chArray.Length * sizeof(char);
						errorCode = RegQueryValueEx(serialCommHkey, keyName, null, ref lpType, chArray, ref dataLen);

						if (errorCode == 0 && lpType == REG_SZ)
						{
							dataLen = dataLen / sizeof(char) - 1; // in bytes without zero terminator

							if (dataLen > 0)
							{
								var valueString = new string(chArray, 0, dataLen);
								r.Add(valueString);
							}
						}
					}
				}
				finally
				{
					RegCloseKey(serialCommHkey);
				}
			}
			catch
			{
				// paranoid catch.. if interop does not find the functions
			}

			return r;
		}
	}
}
