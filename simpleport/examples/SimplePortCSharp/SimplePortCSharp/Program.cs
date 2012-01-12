using System.Runtime.InteropServices;
using System;


    [StructLayout(LayoutKind.Sequential, Pack=1)]
    public struct Msimpleport
    {
        public Int32 usb_handle;
    }

    struct simpleportDLL {
    [DllImport("simpleport.dll")]
    static public extern void simpleport_open(ref Msimpleport m);
    [DllImport("simpleport.dll", CallingConvention = CallingConvention.Cdecl)]
    static public extern void simpleport_close(ref Msimpleport m);
    [DllImport("simpleport.dll", CallingConvention = CallingConvention.Cdecl)]
    static public extern void simpleport_set_pin_dir(ref Msimpleport m, Int32 pin, Int32 value);
    [DllImport("simpleport.dll", CallingConvention = CallingConvention.Cdecl)]
    static public extern void simpleport_set_pin(ref Msimpleport m, Int32 pin, Int32 value);
    [DllImport("simpleport.dll", CallingConvention = CallingConvention.Cdecl)]
    static public extern int simpleport_get_pin(ref Msimpleport m, Int32 pin);
    [DllImport("simpleport.dll", CallingConvention = CallingConvention.Cdecl)]
    static public extern int simpleport_get_port(ref Msimpleport m);
    [DllImport("simpleport.dll", CallingConvention = CallingConvention.Cdecl)]
    static public extern void simpleport_set_port(ref Msimpleport m, Int32 value, Int32 mask);
    [DllImport("simpleport.dll", CallingConvention = CallingConvention.Cdecl)]
    static public extern void simpleport_set_direction(ref Msimpleport m, Int32 direction);        
    };


    class Test
    {
        static void Main()
        {       
            Msimpleport mysimpleport = new Msimpleport();
            simpleportDLL.simpleport_open(ref mysimpleport);

            //simpleportDLL.simpleport_set_pin_dir(ref mysimpleport, 11, 1);

            // Port Funktionen
            simpleportDLL.simpleport_set_direction(ref mysimpleport, 0x1ff); 


            
            System.Console.WriteLine("open ready");
            
            int count = 4;
            //for (int i = 1; i <= count; i++)
            while(true)
            {
                //System.Threading.Thread.Sleep(1000);   
                //simpleportDLL.simpleport_set_pin(ref mysimpleport, 11, 0);
                simpleportDLL.simpleport_set_port(ref mysimpleport, 0x1fe,0x1ff);
                simpleportDLL.simpleport_set_port(ref mysimpleport, 0, 0x1ff);
                //System.Threading.Thread.Sleep(1000);
                //simpleportDLL.simpleport_set_pin(ref mysimpleport, 11, 1);
            }
            
            simpleportDLL.simpleport_set_pin(ref mysimpleport, 11, 0);

            simpleportDLL.simpleport_close(ref mysimpleport);

        }
    }
