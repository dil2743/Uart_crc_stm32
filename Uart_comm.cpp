/* Uart_comm.cpp
* This file can be used to communcate over serial terminal.
* Info is given on bootup screen.
* To genrate 32 bit CRC for ISO 3309 polynomial 
*
* Auther: Dilshad Khan
* 
*/
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <iomanip>
#include <signal.h>
#include <time.h>
// Linux headers
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
using namespace std;

#define MAX_RETRY_ON_NACK 		3
#define MAX_RX_BUFFER_SIZE 		50
bool PRINT_DEBUG_DATA = false ;
const int ACK_VALUE			=	0x77;
const int NACK_VALUE		=	0x99;
const int MIN_DATA_RANGE	=	0;
const int MAX_DATA_RANGE	=	100;
const std::string DEFAULT_SERIAL_ADDRESS = "/dev/ttyACM2";
enum comm_type { COMM_8N1 , COMM_7E1, COMM_7O1 , COMM_7S1};
struct termios options;
int serial_terminal ; // trminal 

/*
 *	@func : Crc32
 *			calculates the crc of given uint32_t data by using ISO 3309 polynomial.
 *	@iput :
 *		arg_1: uint32_t Crc - previous calculated value of Crc
 *		arg_2: uint32_t Data - which crc is to be calculated
 *	@output :
 *		uint32_t : calculated crc value
*/	 
uint32_t Crc32(uint32_t Crc, uint32_t Data)
{
  int i;
 
  Crc = Crc ^ Data;
 
  for(i=0; i<32; i++)
    if (Crc & 0x80000000)
      Crc = (Crc << 1) ^ 0x04C11DB7; // Polynomial used in STM32
    else
      Crc = (Crc << 1);
 
  return(Crc);
}
/*
 *	@func : calculate_checksun
 *			calculates the crc of given uint32_t data_buffer data by using ISO 3309 polynomial.
 *	@iput :
 *		arg_1: uint32_t Crc - previous calculated value of Crc
 *		arg_2: uint32_t[] pBuffer - which crc is to be calculated
 *		arg_3: uint32_t BufferLength - length of pBuffer 
 *	@output :
 *		uint32_t : calculated crc value
*/
uint32_t calculate_checksun(uint32_t Crc, uint32_t pBuffer[], uint32_t BufferLength){
	uint32_t index; 
	for ( index = 0U; index < BufferLength; index++)
	{
		uint32_t temp_ = pBuffer[index];
		Crc =  Crc32(Crc,temp_);
		if(PRINT_DEBUG_DATA){
				char temp[50];
				sprintf(temp, "%x CRC calculate in HEX  %08X \n",temp_,Crc);
				std::cout <<  temp ;
		}
	}
	return Crc ;
} 
/* @func : charHexInt
			to convert charater into into equivalent int representation
	@input :
		arg_1: char c - a character 
	@output:
		int : equivalent intiger value
*/		
static int charHexInt(char c){
    switch (c){
      case '0': return 0;
      case '1': return 1;
      case '2': return 2;
      case '3': return 3;
      case '4': return 4;
      case '5': return 5;
      case '6': return 6;
      case '7': return 7;
      case '8': return 8;
      case '9': return 9;
      case 'A':
      case 'a':
       			return 10;
      case 'B':
      case 'b':
       			return 11;
      case 'C':
      case 'c':
        		return 12;
      case 'D':
      case 'd':
       			return 13;
      case 'E':
      case 'e':
       			return 14;
      case 'F':
      case 'f':
       			return 15;
      default :
		std::cout << "Wrong Input to converter function @FatalError : Error not trapped";
        return -1;
    }
  }
/*
 *	@func : open_port()
 *			Open serial port.
 *	@input :
 *		args_1 : const char* port_address - path of serial port
 *	@output : 
 *		Returns the file descriptor on success or -1 on error.
*/
int open_port(const char * port_address)
{
  int serial_terminal; /* File descriptor for the port */

  serial_terminal = open(port_address, O_RDWR | O_NOCTTY | O_NDELAY);
  if (serial_terminal == -1)
  {
   /*
    * Could not open the port.
    */
    cout<< ("open_port: Unable to open %s ",port_address) ;
  }
  else
    fcntl(serial_terminal, F_SETFL, 0);

  return (serial_terminal);
}
/*
 * @func : init_serial_terminal
 *		call this function to setup the parameters of serial interface
 *	@input :
 *		args_1 : unsigned int baud_rate - baud rate of serial communication
 *		args_2 : enum comm_type comm - serial setting in terms of number of bits,ParetyBit and Stop bit
 *	@output : None
*/
void init_serial_trminal(unsigned int baud_rate , comm_type comm ){
	tcgetattr(serial_terminal, &options); // Get the current options for the port...
	cfsetispeed(&options, baud_rate); // Set the baud rates baud_rate'
	cfsetospeed(&options, baud_rate);
	options.c_cflag |= (CLOCAL | CREAD); // Enable the receiver and set local mode...
	/* set raw input,timeout */
	options.c_lflag     &= ~(ICANON | ECHO | ECHOE | ISIG);
	options.c_oflag     &= ~OPOST;
	options.c_cc[VMIN]  = 50; // input wait time MAX
	options.c_cc[VTIME] = 50; // output MAX time
	options.c_cflag &= ~CSIZE; /* Mask the character size bits */
	options.c_cflag |= CS8;    /* Select 8 data bits */
	switch(comm){
	case COMM_8N1:
	// No parity (8N1):
		options.c_cflag &= ~PARENB ;
		options.c_cflag &= ~CSTOPB ;
		options.c_cflag &= ~CSIZE; 
		options.c_cflag |= CS8;
		break;
	case COMM_7E1:
	// Even parity (7E1):
		options.c_cflag |= PARENB ;
		options.c_cflag &= ~PARODD;
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS7;
		break;
	case COMM_7O1:
	// Odd parity (7O1):
		options.c_cflag |= PARENB  ;
		options.c_cflag |= PARODD;
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS7;
		break;
	case COMM_7S1:
	// Space parity is setup the same as no parity (7S1):
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8;
		break	;
	}	
	tcsetattr(serial_terminal, TCSANOW, &options);// Set the new options for the port...
	sleep(1); // 2 required to make flush work, for some reason
	tcflush(serial_terminal,TCIOFLUSH);
}
/*
 *	@func : send_data
 *			send data to serial and check in blocking loop (for timeout duration options.c_cc[VMIN]) 
 *			for acknowledge
 * 	@input :
 * 		arg_1 :char * data - data that is to be sent
 * 		arg_2 : size_t data_size - size of data that needed to be sent over serial
 * 	@output :
 * 		int -
 * 			 1 :success (data sent and ack received )
 * 			 0 :need to resend data (NACK received)
 * 			-1 :unable to either send or received (process ends here as error is not known)
 */
int send_data(char * data , size_t data_size){
	int counter = write(serial_terminal,data,data_size);
	if(counter < 0 ){
		cout << "@Error:write() of 4 bytes failed!\n " ;
		return -1;
	}

	/* check for ack if it fails then retry for MAX allowed time and then after give up */
	char read_buf[1] = {'\0'};
	int num_bytes = read(serial_terminal, &read_buf, sizeof(read_buf));
	if (num_bytes < 0) {
      printf("@Error:Reading No Ack/Nack received : %s", strerror(errno));
      return -1;
  	}
	if( PRINT_DEBUG_DATA){
    	printf("@info:Read %i bytes. Received message: %s\n", num_bytes, (read_buf[0] == 0x77 )?"ACK":"NACK" );
		if(read_buf[0] == 0x77){
			cout << "@info:Data sent successfully \n";
			return 1;
		}else if(read_buf[0] == NACK_VALUE){
			cout << "@Error:In data transmission \n";
			return 0 ;
		}
	}
	if(read_buf[0] == ACK_VALUE)  return 1;
	else if(read_buf[0] == NACK_VALUE ) return 0 ;
	return -1 ;  
}
void print_info_once(void){
	cout << "     ------------------------------------------------------------------\n";
	cout << "     |                      Info Section                              |\n";
	cout << "     | INPUT  : You can enter integer value between 0 to 100          |\n";
	cout << "     | OUTPUT : There can be 4 types of output based on input         |\n";
	cout << "     |    A: If input divisible by 4 then output = \"Rightbot\"         |\n";
	cout << "     |    B: If input divisible by 7 then output = \"Labs\"             |\n";
	cout << "     |    C: If input divisible by 4 & 7 output = \"Rightbot Pvt Ltd\"  |\n";
	cout << "     |    D: If input not divisible by neither                        |\n";
	cout << "     |          4 nor 7 then output will be the number itself         |\n";
	cout << "     |	   *Along with this, the output is shown on hardware where    |\n";
	cout << "     |     an led can be seen on blinking with 1Hz rate and with a    |\n";
	cout << "     |     brightness level equal to the number as percentage         |\n";
	cout << "     | INFO: If two input are given back to back then result will     |\n";
	cout << "     |       be displayed once the last input is processed            |\n";
	cout << "     | INFO: two args can be passed A) PortAddress B)DebugFlag        |\n";
	cout << "     | 	   enable verbose mode pass V as second argument           |\n";
	cout << "     | EXIT: Press ctrl+c for exit                                    |\n";
	cout << "     ------------------------------------------------------------------\n";
}
void signal_callback_handler(int signum) {
   cout << "Caught signal " << signum << endl;
   close(serial_terminal);
   // Terminate program
   exit(signum);
}
/* @func: rec_input
 * @input: Void
 * @output: int - received data
 */ 
int rec_input(void){
	int input_data = -1 ;
	static bool is_data_received = 1;
	int nowTime = clock();
	do{
		cin >> input_data;
	}while( (input_data < 0 || input_data > 100) && (clock() - nowTime ) < 1  );
	return  input_data;
}
/*
 * handle CRC for incomming data
 */
 bool validate_crc(char * read_buf , int num_bytes){
	    char rec_crc[2][9],  rec_data[100] = { '0' }; // rec_crc[0] - holds the incomming crc and rec_crc[1] holds computed one
		bool crc_validation_status = 0;
		strncpy(rec_crc[0],&read_buf[num_bytes-8],8); // sepratte crc from received stream
		rec_crc[0][8] = '\0';
		strncpy(rec_data,&read_buf[0],num_bytes-8); // seprate data from received stream
		rec_data[num_bytes-8 + 1] = '\0';
		if(PRINT_DEBUG_DATA){
		std::cout << "@info:DataReceived "<<endl;
		std::cout << "@info:Raw " << read_buf <<endl;
		std::cout << "@info:rec_data " << rec_data <<endl;
		std::cout << "@info:rec_crc " << rec_crc[0] <<endl;
	}
		int index = 0 , BufferLength = num_bytes-8 ;
		uint32_t *pBuffer = (uint32_t*)rec_data; 
		uint32_t data_crc = 0xffffffff;
		for (index = 0U; index < BufferLength; index++)
		{
			data_crc = calculate_checksun(data_crc,&pBuffer[index],1); // calculate crc 
		}
		sprintf(rec_crc[1],"%.8x",data_crc); // convert calculated crc to string 
		rec_crc[1][8] = '\0';
		if(strcmp(rec_crc[0],rec_crc[1]) == 0 ){ // compare received crc and computed crc
			crc_validation_status = 1 ;
		}else{
			crc_validation_status = 0 ;
		}
	if(PRINT_DEBUG_DATA){
		std::cout << "@info:calculated_crc " <<rec_crc;
	}
	if(crc_validation_status){
		printf("@Output:\t\t\t %s\n",rec_data); // display the out put id received stream is valid 
		// write(serial_terminal,(char*)&ACK_VALUE,1);
		return 1;
	}
	else{
		printf("@Output:CRC validation failed Resend command\n");
		// write(serial_terminal,(char*)&NACK_VALUE,1);
	}
	return 0;
 }
bool received_serail_data(void){
		char read_buf [MAX_RX_BUFFER_SIZE];
		memset(&read_buf, '\0', sizeof(read_buf));
		int num_bytes = read(serial_terminal, &read_buf, sizeof(read_buf));
		if (num_bytes < 0) {
			printf("Error reading: %s", strerror(errno)); // n data received till specifed timeout 
			return 0;
		}
		return validate_crc(read_buf,num_bytes); // data received 
}
int main(const int nArg, const char *aArg[]){
	signal(SIGINT, signal_callback_handler);
	print_info_once();
	 // take serial address as an argument if it is not there then move with defalut one
	std::string port_address = (nArg > 1)
        ? &aArg[1][0]
        : DEFAULT_SERIAL_ADDRESS
        ;
	std::string debug_flag = (nArg > 2)
        ? &aArg[2][0]
        : "N"
        ;
	if(debug_flag == "V" || debug_flag == "v"){
		 PRINT_DEBUG_DATA = true;
		 cout << "verbose mode" << endl;
	}

	serial_terminal = open_port(port_address.c_str()); 

	if(serial_terminal == -1){
		cout << "\nFatalError:COMM Open error at "<< port_address << endl;
		std::string port;
		int fd;
		for(int i = 0; i < 256; ++i)
		{
			port.clear();
			port.append("/dev/ttyUSB");
			port.append(std::to_string(i));
			fd = open(port.c_str(), O_RDWR | O_NOCTTY );
			if(fd != -1)
			{
				cout << "@info:"<< port <<" avalable"<<endl;	
			}
			port.clear();
			port.append("/dev/ttyACM");
			port.append(std::to_string(i));
			fd = open(port.c_str(), O_RDWR | O_NOCTTY );
			if(fd != -1)
			{
				cout << "@info:"<< port <<" avalable"<<endl;	
			}

		}
		return 1;
	}
	init_serial_trminal(9600,COMM_8N1); // init serial 
	auto startTime = clock();
	cout << "@Input:Type{Int} Range{0-100}\n" ;

	while(true){
		int input_data = rec_input();
		if(input_data >= MIN_DATA_RANGE && input_data <= MAX_DATA_RANGE){
			uint32_t temp_data = input_data ;
			uint32_t temp_crc = calculate_checksun(0xffffffff,&temp_data, 1);
			char DATA = temp_data; 
			char CRC_BYTE_3 = (temp_crc  >> 24 & 0xff);
			char CRC_BYTE_2 = (temp_crc  >> 16 & 0xff);
			char CRC_BYTE_1 = (temp_crc  >> 8 & 0xff);
			char CRC_BYTE_0 = (temp_crc  >> 0 & 0xff);
			char my_data[5] = {DATA, CRC_BYTE_3 ,CRC_BYTE_2 ,CRC_BYTE_1, CRC_BYTE_0};
			uint8_t retry_counter = 1 , n = 0 ;
			while(retry_counter < MAX_RETRY_ON_NACK && n == 0){
				sleep(1);
				tcflush(serial_terminal,TCIOFLUSH);
				n = send_data(my_data, 5);
				retry_counter++;
				if(PRINT_DEBUG_DATA){
					if(n != 1)
						cout <<"@info:Tx error Resending data"<<endl;
				}
			}
			if(n == -1 ) {
				cout << "\nFatal Error: exit called not able to Tx or Rx \n";
				return 1;
			}
			bool status = received_serail_data();
		}else{
			 cout << "@Input:Error-Type{Int} Range{0-100}\n" ;
			}
	}
	close(serial_terminal);
	return EXIT_SUCCESS; 
}
