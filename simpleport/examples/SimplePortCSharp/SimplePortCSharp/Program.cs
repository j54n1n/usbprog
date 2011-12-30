using System.Runtime.InteropServices;
using System;



    [StructLayout(LayoutKind.Sequential, Pack=1)]
    public struct Msimpleport
    {
        public Int32 usb_handle;
    }

    struct simpleportDLL {
    [DllImport("simpleport.dll",CallingConvention=CallingConvention.Cdecl)]
    static public extern void simpleport_open(ref Msimpleport m);
    [DllImport("simpleport.dll", CallingConvention = CallingConvention.Cdecl)]
    static public extern void simpleport_close(ref Msimpleport m);
    [DllImport("simpleport.dll", CallingConvention = CallingConvention.Cdecl)]
    static public extern void simpleport_set_pin_dir(ref Msimpleport m, Int32 pin, Int32 value);
    [DllImport("simpleport.dll", CallingConvention = CallingConvention.Cdecl)]
    static public extern void simpleport_set_pin(ref Msimpleport m, Int32 pin, Int32 value);        
    };


    class Test
    {
        static void Main()
        {       
            Msimpleport mysimpleport = new Msimpleport();
            simpleportDLL.simpleport_open(ref mysimpleport);

            simpleportDLL.simpleport_set_pin_dir(ref mysimpleport, 11, 1);
            
            System.Console.WriteLine("open ready");
            
            int count = 4;
            for (int i = 1; i <= count; i++)
            {
                System.Threading.Thread.Sleep(1000);   
                simpleportDLL.simpleport_set_pin(ref mysimpleport, 11, 0);
                System.Threading.Thread.Sleep(1000);
                simpleportDLL.simpleport_set_pin(ref mysimpleport, 11, 1);
            }
            
            simpleportDLL.simpleport_set_pin(ref mysimpleport, 11, 0);

            simpleportDLL.simpleport_close(ref mysimpleport);

        }
    }
