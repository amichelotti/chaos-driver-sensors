//
//  test_serial.cpp
// 
//
//
//
//


#include <common/serial/core/SerialChannelFactory.h>
#include <boost/program_options.hpp>
#include <boost/regex.hpp>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <common/debug/core/debug.h>
#include <boost/regex.hpp>
#include <string>
#ifdef CHAOS
#include <chaos_metadata_service_client/ChaosMetadataServiceClient.h>
using namespace chaos::metadata_service_client;

#endif
 namespace po=boost::program_options;

#define DEFAULT_TIMEOUT OCEM_DEFAULT_TIMEOUT_MS


static int default_timeout = DEFAULT_TIMEOUT;
using boost::regex;

int main(int argc, const char *argv[])
{

std::string ver;
    float maxcurrent=100;
    float maxvoltage=10;
    bool span=false;
    bool interactive=false;
    int slave_id;
    std::string dev;

 
    
      ChaosMetadataServiceClient::getInstance()->getGlobalConfigurationInstance()->addOption("ip:port", po::value<std::string>(&dev), "The tcp2serial address:port");
      
      ChaosMetadataServiceClient::getInstance()->getGlobalConfigurationInstance()->addOption("timeout,t", po::value<int>(&default_timeout)->default_value(DEFAULT_TIMEOUT), "timeout in ms ");

      ChaosMetadataServiceClient::getInstance()->init(argc, argv);

      
   

  //////
 
   ::common::serial::AbstractSerialChannel_psh channel;
   channel=::common::serial::SerialChannelFactory::getChannel(dev,9600,0,8,1);

  if(span){
    std::cout<<"finding device on the bus"<<std::endl;
    int id=0;
    int found=0;
   
    for(;id<32;id++){
      ::common::powersupply::AbstractPowerSupply *ps= new ::common::powersupply::OcemE642X("OcemProtocolScheduleCFQ",channel,id,maxcurrent,maxvoltage);
      if(ps==NULL){
	std::cout<<"## cannot initialize resources"<<std::endl;
	return -2;
	
      }
      std::cout<<"Trying address:"<<id<<std::endl;
      if(ps->init()==0){
    	  std::cout<<"found:"<<id<<std::endl;
    	  found ++;
      }
      delete ps;
    }
    return found;
  }
 /* if(vm.count("raw")){
    common::serial::OcemProtocol* oc= new common::serial::OcemProtocol(param.c_str());
    if(oc) 
      raw_test(oc);

    delete oc;
    return 0;
  }
*/
  

  
  printf("Connecting to slave %d, via \"%s\"... \n",slave_id,dev.c_str());
  common::powersupply::OcemE642X *ps= new common::powersupply::OcemE642X("OcemProtocolScheduleCFQ",channel,slave_id,maxcurrent,maxvoltage);

  if(ps){
    std::cout<<"Initializing driver"<<std::endl;

    if(ps->init()!=0){
      printf("## cannot initialise power supply\n");
      return -1;
    }
  }
  printf("OK\n");
  if(interactive){
    if(  ps->getSWVersion(ver,default_timeout)!=0){
      printf("## cannot get SW version \n");
      return -3;
    }
    printf("Version:%s\n",ver.c_str());
    while(1){
      float curr,volt,sp;
      int pol,stat;
      std::string state;
      int ch,ret;
      uint64_t ev;
      if((ret= ps->getCurrentOutput(&curr,default_timeout))<0){
	printf("\n## error retrieving current, ret %d\n",ret);
      }
      if((ret=ps->getVoltageOutput(&volt,default_timeout))<0){
	printf("\n## error retrieving voltage, ret %d\n",ret);
      }
      if((ret=ps->getCurrentSP(&sp,default_timeout))<0){
	printf("\n## error retrieving current set point, ret %d\n",ret);
      }
      if((ret=ps->getPolarity(&pol,default_timeout))<0){
	printf("\n## error retrieving polarity, ret %d\n",ret);
      }
      if((ret=ps->getState(&stat,state,default_timeout))<0){
	printf("\n## error retrieving State, ret %d\n",ret);
      }
      if((ret=ps->getAlarms(&ev,default_timeout))<0){
	printf("\n## error retrieving Alarms, ret %d\n",ret);
      } 
      printf("A:%3.4f,V:%2.2f,SetPoint('1'):%3.4f,Polarity('2'):%c,State('3'):(0x%x)\"%s\", alarms(reset '4'):x%lX\r",curr,volt,sp,(pol>0)?'+':(pol<0)?'-':'0',stat,state.c_str(),ev);
      fflush(stdout);
      if(check_for_char() && (ch=getchar())){
	if(ch == 'q'){
	  printf("quitting....\n");
	  break;
	}
	if(ch=='1'){
	  float max,min;
	  float cur;
	  ps->getMaxMinCurrent(&max,&min);
	  printf("\nSet current point [%3.4f:%3.4f]:",min,max);
	  scanf("%f",&cur);
	  printf("\n");
	  if( (ret=ps->setCurrentSP(cur,default_timeout))<0){
	    printf("\n## error setting current ret %d\n",ret);
	  } else {
	    if( (ret=ps->startCurrentRamp(default_timeout))<0){
	      printf("\n## error starting ramp%d\n",ret);
	    }
	  }
	}
      
	if(ch=='2'){
	  int polarity;
	  printf("\nSet Polarity [<0 negative,>0 positive, 0 open]:");
	  scanf("%d",&polarity);
	  printf("\n");
	  if( (ret=ps->setPolarity(polarity,default_timeout))<0){
	    printf("\n## error setting polarity ret %d\n",ret);
	  } 
	}

	if(ch=='4'){
	  uint64_t alm=0;
	  printf("\nResetting events\n");

	  if( (ret=ps->resetAlarms(alm,default_timeout))<0){
	    printf("\n## error resetting alarms ret %d\n",ret);
	  } 
	}
	    
	if(ch=='3'){
	  int polarity;
	  printf("\nSet State [1 Power ON, 2 StandBy , 3  Shutdown (the communincation could drop):");
	  scanf("%d",&polarity);
	  printf("\n");
	  if(polarity==1){
	    if( (ret=ps->poweron(default_timeout))<0){
	      printf("\n## error setting power on ret %d\n",ret);
	    }
	  }
	
	  if(polarity==2){
	    if( (ret=ps->standby(default_timeout))<0){
	      printf("\n## error setting standby ret %d\n",ret);
	    }
	  }
	
	  if(polarity==3){
	    if( (ret=ps->shutdown(default_timeout))<0){
	      printf("\n## error setting OFF ret %d\n",ret);
	      
	    }
	  }
	
	}
      
      }
    }
  } else {
    char stringa[1024];
    char cmd[256],val[256],val1[256];
    printf("waiting commands (HELP for command list)\n");
    while(fgets(stringa,sizeof(stringa),stdin)){
      int ret;
      uint64_t tm;
      char *t=stringa;
      convToUpper(t);
      tm = common::debug::getUsTime();
      if(sscanf(stringa,"%s %s %s",cmd,val,val1)==3){
	if(!strcmp(cmd,"OPEN")){
	  if(ps!=NULL){
	    printf("## device already open \"CLOSE\" before\n");
	    continue;
	  } else {
		printf("NOT IMPLEMENTED ANY MORE\n");
		continue;
		ps= new common::powersupply::OcemE642X("OcemProtocolScheduleCFQ",channel,slave_id,maxcurrent,maxvoltage);
	    if(ps==NULL){
	      printf("cannot allocate resources for \"%s:%s\"\n",val,val1);
	      continue;
	    } else {
	      printf("Connecting to slave %s, via \"%s\"... \n",val1,val);
	      if(ps->init()!=0){
		printf("## cannot initialise power supply on \"%s:%s\"\n",val,val1);
		delete ps;
		ps = NULL;
		continue;
	      }
	    }
	  }
	  
	} else if(!strcmp(cmd,"SLOPE")){
	  float up = atof(val);
	  float down=atof(val1);
	  printf("setting current SLOPE UP %4.4f DOWN %4.4f\n",up,down);
	  if( (ret=ps->setCurrentRampSpeed(up,down,default_timeout))<0){
	    printf("## error setting SLOPE ret %d\n",ret);
	    continue;
	  } 
	} else {
	  printf("## syntax error, syntax is SLOPE <UP> <DOWN>\n"); 
	  continue;
	}
      
      } else if(sscanf(stringa,"%s %s",cmd,val)==2){
	int ival = atoi(val);
	float fval = atof(val);
        if(!strcmp(cmd,"SELECT")){
            int ret;
            ret=ps->send_command(val,1000,0);
            if(ret<0){
                printf("## error SELECT %s ret %d\n",val,ret);
                continue;
            } 
            
        } else if(!strcmp(cmd,"POL")){

	  printf("setting polarity to %d\n",ival);
	  if( (ret=ps->setPolarity(ival,default_timeout))<0){
	    printf("## error setting polarity ret %d\n",ret);
	    continue;
	  } 
	} else if(!strcmp(cmd,"TEST_ONSTBY")){
		if((ret=test_stbyon(ps,ival))!=0){
			  printf("## error performing test ret %d\n",ret);
				    continue;
		}

	} else if(!strcmp(cmd,"SP")){
	  printf("setting current SP to %4.4f\n",fval);
	  if( (ret=ps->setCurrentSP(fval,default_timeout))<0){
	    printf("## error setting SP ret %d\n",ret);
	    continue;
	  } 
	} else {
	  printf("## syntax error\n"); 
	  continue;
	}
      }  else if(sscanf(stringa,"%s",cmd)==1){
	if(!strcmp(cmd,"RAMP")){
	  printf("start ramp..\n");
	  if( (ret=ps->startCurrentRamp(default_timeout))<0){
	    printf("## error starting ramp ret %d\n",ret);
	    continue;
	  } 
	} else if(!strcmp(cmd,"STANDBY")){
	  printf("setting standby\n");
	  if( (ret=ps->standby(default_timeout))<0){
	    printf("## error STANDBY ret %d\n",ret);
	    continue;
	  } 
	} else if(!strcmp(cmd,"POLL")){
          char buffer[1024];
          int tim;
	  printf("executing poll\n");
	  if( (ret=ps->receive_data(buffer,sizeof(buffer),5000,&tim))<0){
	    printf("## error POLLING ret %d\n",ret);
	    continue;
	  }  else {
              printf("->\"%s\"\n",buffer);
          }
	} else if(!strcmp(cmd,"ON")){
	  printf("setting poweron\n");
	  if( (ret=ps->poweron(default_timeout))<0){
	    printf("## error POWERON ret %d\n",ret);
	    continue;
	  } 
	} else if(!strcmp(cmd,"OFF")){
	  printf("setting shutdown\n");
	  if( (ret=ps->shutdown(default_timeout))<0){
	    printf("## error shutdown ret %d\n",ret);
	    continue;
	  } 
	} else if(!strcmp(cmd,"RSTALARMS")){
	  printf("resetting alarms\n");
	  if( (ret=ps->resetAlarms(0,default_timeout))<0){
	    printf("## error resetting allarms ret %d\n",ret);
	    continue;
	  } 
	} else if(!strcmp(cmd,"GETCURRENT")){
	  float curr;
	  if( (ret=ps->getCurrentOutput(&curr,default_timeout))<0){
	    printf("## error getting current ret %d\n",ret);
	    continue;
	  } else {
	    printf("* %4.4f\n",curr);
	  }
	} else if(!strcmp(cmd,"GETVOLTAGE")){
	  float curr;
	  if( (ret=ps->getVoltageOutput(&curr,default_timeout))<0){
	    printf("## error getting voltage ret %d\n",ret);
	    continue;
	  } else {
	    printf("* %4.4f\n",curr);
	  }
	} else if(!strcmp(cmd,"GETALARMS")){
	  uint64_t curr;

	  if( (ret=ps->getAlarms(&curr,default_timeout))<0){
	    printf("## error getting alarms ret %d\n",ret);
	    continue;
	  } else {
	    std::string ret=common::powersupply::AbstractPowerSupply::decodeEvent((common::powersupply::PowerSupplyEvents)curr);
	    printf("* %.16lx \"%s\"\n",curr,ret.c_str());
	  }
	} else if(!strcmp(cmd,"GETSTATE")){
	  int stat;
	  std::string desc;
	  if( (ret=ps->getState(&stat,desc,default_timeout))<0){
	    printf("## error getting state ret %d\n",ret);
	    continue;
	  } else {
	    printf("* 0x%.8x (%s)\n",stat,desc.c_str());
	  }
	} else if(!strcmp(cmd,"GETPOL")){
	  int stat;

	  if( (ret=ps->getPolarity(&stat,default_timeout))<0){
	    printf("## error getting polarity ret %d\n",ret);
	    continue;
	  } else {
	    printf("* \"%s\"\n",stat>0?"POS":stat<0?"NEG":"OPN");
	  }
	} else if(!strcmp(cmd,"GETVER")){
	  std::string desc;
	  if( (ret=ps->getHWVersion(desc,default_timeout))<0){
	    printf("## error getting state ret %d\n",ret);
	    continue;
	} else {
	    float cmax,cmin,vmax,vmin;
	    ps->getMaxMinCurrent(&cmax,&cmin);
	    ps->getMaxMinVoltage(&vmax,&vmin);
	    printf("* %s (max current %4.4f, max voltage %4.4f )\n",desc.c_str(),cmax,vmax);
	  }
	} else if(!strcmp(cmd,"GETSP")){
	  float curr;
	  if( (ret=ps->getCurrentSP(&curr,default_timeout))<0){
	    printf("## error getting SP ret %d\n",ret);
	    continue;
	  } 
	  printf("* %4.4f\n",curr);
	} else if(!strcmp(cmd,"QUIT")){
	  printf("quitting\n");
	  break;
	} else if(!strcmp(cmd,"CLOSE")){
	  delete ps;
	  ps = NULL;

	} else if(!strcmp(cmd,"HELP")){
	  printCommandHelp();

	} else {
	  printf("## syntax error\n");
	  continue;
	}
      }
      printf("* OK [%.8lu ms]\n",(common::debug::getUsTime()-tm)/1000);
    }
  }
  delete ps;
  printf("* OK\n");
  return 0;
}

