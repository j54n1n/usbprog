using System.Runtime.InteropServices;
using System;



    [StructLayout(LayoutKind.Sequential, Pack=8)]
    public struct Msimpleport
    {
        //public IntPtr usb_handle;
         public IntPtr usb_handle;
        //byte[] usb_handl
    }

    struct simpleportDLL {
   [DllImport("simpleport.dll")]
    static public extern void simpleport_open(ref Msimpleport m);
   [DllImport("simpleport.dll")]
    static public extern void simpleport_close(ref Msimpleport m);
   [DllImport("simpleport.dll")]
    static public extern void simpleport_set_pin_dir(ref Msimpleport m, Int32 pin, Int32 value);        
    };


    /*[DllImport("simpleport.dll")]
    private static extern void simpleport_open(
        ref Msimpleport s
        );
   
    [DllImport("simpleport.dll")]
    private static extern void simpleport_close(
     ref simpleport s
    );

    [DllImport("simpleport.dll")]
    public static extern int simpleport_set_pin_dir(
     ref simpleport s,
     int pin,
     int dir
    );

    [DllImport("simpleport.dll")]
    private static extern int simpleport_set_pin(
     ref simpleport s,
     int pin,
     int value
    );
    */

    class Test
    {
        static void Main()
    {

            
            Msimpleport mysimpleport = new Msimpleport();
            simpleportDLL.simpleport_open(ref mysimpleport);
            System.Console.WriteLine("open ready");
            
            simpleportDLL.simpleport_set_pin_dir(ref mysimpleport, 11, 0);
            System.Threading.Thread.Sleep(500);
            simpleportDLL.simpleport_set_pin_dir(ref mysimpleport, 11, 1);
            System.Threading.Thread.Sleep(500);
            simpleportDLL.simpleport_set_pin_dir(ref mysimpleport, 11, 0);
            simpleportDLL.simpleport_close(ref mysimpleport);

    }
    }
