#include <stdio.h>
#ifdef _WIN32
        #include <windows.h>
#else
        #include <unistd.h>
#endif
#include "simpleport.h"

int main()
{
  struct simpleport sp_handle;

  printf("libsimpleport Demo\n");

  /* open connection to simpleport */
  simpleport_open(&sp_handle);
	
  if(&sp_handle==0)
    printf("unable to open device\n");
#if 0
	int i;

  simpleport_set_direction(sp_handle,0xFF);

  
  for(i=0;i<4;i++){
    simpleport_set_port(sp_handle,0x80,0xFF);
    #ifdef WIN32
	  Sleep(100);
	#else
      usleep(1000*1000);
	#endif
    simpleport_set_port(sp_handle,0x00,0xFF);
    #ifdef WIN32
	  Sleep(100);
	#else
      usleep(1000*1000);
	#endif  
	}

  simpleport_set_pin_dir(sp_handle,11,1);
  //for(i=0;i<4;i++){
  while(1){
    simpleport_set_pin(sp_handle,11,1);
    #ifdef WIN32
	  Sleep(100);
	#else
      usleep(1000*1000);
	#endif    
	simpleport_set_pin(sp_handle,11,0);
    #ifdef WIN32
	  Sleep(100);
	#else
      usleep(1000*1000);
	#endif  
	}

  int j;
  for(j=0;j<3;j++) {
    for(i=0;i<200;i++){
      simpleport_set_pin(sp_handle,11,1);
      simpleport_set_pin(sp_handle,11,0);
      simpleport_set_pin(sp_handle,11,0);
      simpleport_set_pin(sp_handle,11,0);
      simpleport_set_pin(sp_handle,11,0);
    }
  
    for(i=0;i<200;i++){
      simpleport_set_pin(sp_handle,11,1);
      simpleport_set_pin(sp_handle,11,1);
      simpleport_set_pin(sp_handle,11,1);
      simpleport_set_pin(sp_handle,11,1);
      simpleport_set_pin(sp_handle,11,0);
    }
  }

  
#endif
//simpleport_close(sp_handle);
  return 0;
}
