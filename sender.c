/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>

#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyS1"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1
#define FLAG 0x7E
#define START 0
#define END 1
#define F 1
#define FA 2
#define FAC 3
#define A 0x03
#define A2 0x01

#define CUA 0x07
#define CSET 0x03
#define Cs0 0x00
#define Cs1 0x02
#define CDISC 0x0B
#define SUCCESS 4
#define RR0 0x05
#define RR1 0x25
#define BPP 10 //BYTES_PER_PACKAGE

#define ESCAPE0 0x7d
#define ESCAPE1 0x5e
#define ESCAPE2 0x5d

volatile int ALARM, GO_ON, ALARMSET, STOPALARM;
char receiveState=0 ;
int c, timeout=3,count=0;
struct termios oldtio,newtio;
char buf[255], cpy[255];
int sum = 0, speed = 0;
int pointer_resbuf=0;



// NEW CODE ----------------------------------------------------------------------------

void alarme()
{
	count++;
	ALARM=TRUE;
	if(count>2){
		printf("Error! Receiver does not respond...\n");
		exit(1);
	}
	else{
		printf("Error! Trying again...\n");
		alarm(timeout);
	}

}

int llwrite(int fd, char* buffer, int length);
int writeSET(int fd);
int checkUA(char* buffer);
int llread(int fd, char* buffer, int i);
void read5bytes(int fd, char* buffer);
int writeDISC(int fd);
int checkDISC(char* buffer);
int writeUA(int fd);
int checkRR(char* buffer, char ns);
int datapacket(char *file, unsigned char *datapacket, int y, int k, char n);
int controlpacket(char* controlpacket, char cc, char* sizeoffile, char* filename);
int min(int a, int b);
int updateSequenceNumber(char n);
int writeI(int fd, char ns, int pointer, char* buffer, int charnumber, char n);

int stuff(char * buffer, int tamanhoString, char * stuffedbuffer, int * tamanho)
{
	int m, count=0;
	int n = 0,i;

	for (m = 0; m < tamanhoString; m++)
	{

		  if (buffer[m] == FLAG)
		  {
			stuffedbuffer[n++] = ESCAPE0;
			stuffedbuffer[n++] = ESCAPE1;
			count++;
		  }
		  else if(buffer[m] == ESCAPE0)
		  {
			stuffedbuffer[n++] = ESCAPE0;
			stuffedbuffer[n++] = ESCAPE2;
			count++;
		  }
		  else
		  {
			stuffedbuffer[n++] = buffer[m];
		  }
		}
	tamanho = n;
	printf("count: %d\n",count);
	return count;
}

int llopen(int porta){
	int fd;
	char nomeporta[11],next=FALSE, isValid;
	switch(porta){
	case 0:
		strcpy(nomeporta,"/dev/ttyS0");
		break;
	case 1:
		strcpy(nomeporta,"/dev/ttyS1");
		break;
	default:
		perror("Porta invalida\n");
		return -1;
	}

	fd = open(nomeporta, O_RDWR | O_NOCTTY);
	if (fd <0){
		return fd;
	}

	if (tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
		perror("tcgetattr");
		exit(-1);
	}

	bzero(&newtio, sizeof(newtio));
	newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;


	/* set input mode (non-canonical, no echo,...) */
	newtio.c_lflag = 0;
	newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
	newtio.c_cc[VMIN]     = 1;   /* blocking read until 5 chars received */

	tcflush(fd, TCIOFLUSH);

	if ( tcsetattr(fd,TCSANOW,&newtio) == -1) {
		perror("tcsetattr");
		exit(-1);
	}

	fcntl(fd,F_SETFL,FNDELAY);
	(void) signal(SIGALRM, alarme);
	printf("Serial port open\n\n");

	unsigned char buffer[5];
	alarm(timeout);

	GO_ON=FALSE;
	ALARM=TRUE;
	while (GO_ON!=TRUE)
	{
		if(ALARM==TRUE){
			writeSET(fd);
			ALARM=FALSE;
			usleep(200000);
			read5bytes(fd,buffer);
			tcflush(fd, TCIOFLUSH);
			isValid = checkUA(buffer);
			if(isValid==TRUE){
				printf("Resposta recebida (UA)\n");
				alarm(0);
				break;
			}
		}
	}
	return fd;
}

int llread(int fd, char* buffer,int i){
	int res;
	res = read(fd,buffer+i,1);
	switch(i){
		case 0:
		if(buffer[i]==FLAG){
			i++;
		}
		break;

		case 1:
		if(buffer[i]!=FLAG){
			i++;
		}
		break;

		case 2:
		if(buffer[i] == F){
			i=1;
		}
		else{
			i++;
		}
		break;

		case FAC:
		do{
			i++;
			res = read(fd,buffer+i,1);
		}while(buffer[i]!=FLAG);
		break;
	}
	return i;
}

int llwrite(int fd, char* buffer, int length){
	int res,i;
	res = write(fd,buffer,length);
	return res;

}

void llclose(int fd){
	char buffer[5];
	int isValid;

	count=0;
	alarm(timeout);
	GO_ON=FALSE;
	ALARM=TRUE;
	while (GO_ON!=TRUE)
	{
		if(ALARM==TRUE){
			writeDISC(fd);
			ALARM=FALSE;
			usleep(200000);
			read5bytes(fd,buffer);
			ALARMSET=FALSE;
			tcflush(fd, TCIOFLUSH);
			isValid = checkDISC(buffer);
			if(isValid==TRUE){
				printf("Resposta recebida (UA)\n");
				alarm(0);
				break;
			}
		}
	}

	writeUA(fd);

	usleep(100000);
	tcsetattr(fd,TCSANOW,&oldtio);
	close(fd);
	printf("\nSerial port closed, exiting...\n");
}

void transfDados(int fd, unsigned char* charsperpacketc, char* filename){
	FILE *file;
	int filesize,i,charsperpacket = atoi(charsperpacketc);
	printf("%d\n",charsperpacket);
	//int charsperpacket=atoi(charsperpacketc);
	//int controlpacketsize = 4+sizeof(filesize)+sizeof(filename);
	//char* resultbuffer[buffersize];
	//char* filesizec;
	//sprintf(filesizec,"%d",filesize);


	file = open(filename, O_RDONLY, S_IWRITE | S_IREAD);
	printf("File openned\n");

	struct stat st;
	stat(filename, &st);
	filesize = st.st_size;
	unsigned char filebuffer[filesize];
	int pointer=0, temppointer;
	printf("Alocados %d bytes para o filebuffer\n",filesize);

	char sizeoffile[10];
	sprintf(sizeoffile,"%d",filesize);

	read(file,filebuffer,filesize);

	printf("File content:\n");
	for(i=0;i<filesize;i++){
		printf("%x ",filebuffer[i]);
	}
	printf("\n\n");

	char isValid = FALSE;
	char ns=0,STOP=FALSE,n=0;
	char* resbuf[5];

	count=0;
	GO_ON=FALSE;
	ALARM=TRUE;
	alarm(timeout);
	while (GO_ON!=TRUE)
	{
		if(ALARM==TRUE){
			writeFirstI(fd,sizeoffile,filename);
			ALARM=FALSE;
			usleep(200000);
			read5bytes(fd,resbuf);
			tcflush(fd, TCIOFLUSH);
			isValid = checkRR(resbuf,ns);
			if(isValid==TRUE){
				printf("Resposta recebida (RR)\n");
				alarm(0);
				break;
			}
		}
	}

	ns=1;

	while(pointer<filesize){
		count=0;
		GO_ON=FALSE;
		ALARM=TRUE;
		alarm(timeout);
		while (GO_ON!=TRUE)
		{
			if(ALARM==TRUE){
				temppointer = writeI(fd,ns,pointer,filebuffer,min(charsperpacket,filesize-pointer),n);
				ALARM=FALSE;
				usleep(200000);
				read5bytes(fd,resbuf);
				tcflush(fd, TCIOFLUSH);
				isValid = checkRR(resbuf,ns);
				if(isValid==TRUE){
					pointer+=temppointer;
					if(pointer>=filesize){
						alarm(0);
						break;
					}
					printf("RR check\n");
					n=updateSequenceNumber(n);
					alarm(0);
					if(ns==0)
						ns=1;
					else
						ns=0;
					break;
				}
			}
		}
	}

	if(ns==0)
		ns=1;
	else
		ns=0;

	count=0;
	count=0;
	GO_ON=FALSE;
	ALARM=TRUE;
	alarm(timeout);
	while (GO_ON!=TRUE)
	{
		if(ALARM==TRUE){
			writeLastI(fd, ns,sizeoffile,filename);
			ALARM=FALSE;
			usleep(200000);
			read5bytes(fd,resbuf);
			tcflush(fd, TCIOFLUSH);
			isValid = checkRR(resbuf,ns);

			if(isValid==TRUE){
				printf("RR check\n");
				alarm(0);
				ns=1;
				break;
			}
		}
	}
}

char getBCC2(char* buffer, int size){
	char res = 0;
	int i;
	for(i=0;i<size;i++){
		res = res^buffer[i];
	}
	return res;
}

int writeSET(int fd){
	int res;
	char buffer[5];
	buffer[0] = FLAG;
	buffer[1] = A;
	buffer[2] = CSET;
	buffer[3] = A^CSET; //BCC
	buffer[4] = FLAG;

	res = llwrite(fd,buffer,5);
	if(res>0)
		printf("Comando enviado (SET)\n");
	else
		exit(1);
	return res;
}

void writeFirstI(int fd, char* sizeoffile, char* filename){
	int i,pointer=4,size;
	unsigned char controlbuf[5+strlen(sizeoffile)+strlen(filename)+20];

	controlbuf[0]=FLAG;
	controlbuf[1]=A;
	controlbuf[2]=0;
	controlbuf[3]=A^0;

	size = controlpacket(controlbuf+pointer, 1, sizeoffile, filename);
	pointer +=size;

	controlbuf[pointer]=getBCC2(controlbuf+4,size);
	controlbuf[++pointer]=FLAG;
	llwrite(fd,controlbuf,++pointer);

	printf("Control packet trama content: ");
	for(i=0;i < pointer; i++){
		printf("%x ",controlbuf[i]);
	}
	printf("\n");
}

void writeLastI(int fd, int ns, char* sizeoffile, char* filename){
	int i,pointer=4,size;
	unsigned char controlbuf[5+strlen(sizeoffile)+strlen(filename)+20];
	int cns=0;

	controlbuf[0]=FLAG;
	controlbuf[1]=A;
	if(ns==0)
	{
		cns=0;
	}
	else
	{
		cns=2;
	}
	controlbuf[2]=cns;
	controlbuf[3]=A^cns;

	size = controlpacket(controlbuf+pointer, 2, sizeoffile, filename);
	pointer +=size;

	controlbuf[pointer]=getBCC2(controlbuf+4,size);
	controlbuf[++pointer]=FLAG;
	llwrite(fd,controlbuf,++pointer);

	printf("Control packet trama content: ");
	for(i=0;i < pointer; i++){
		printf("%x ",controlbuf[i]);
	}
	printf("\n");
}

int writeI(int fd, char ns, int pointer, char* buffer, int charnumber, char n){
	int pos, *resultsize, newpointer;
	unsigned char resultbuf[charnumber*2+10];
	resultbuf[0] = FLAG;
	resultbuf[1] = A;
	if(ns==0)
	{
		resultbuf[2] = Cs0;
		resultbuf[3] = A^Cs0; //BCC1
	}
	else
	{
		resultbuf[2] = Cs1;
		resultbuf[3] = A^Cs1; //BCC1
	}
	pos=4;

	resultsize=0;

	/*pointer = datapacket(buffer, resultbuf+pos, pointer, charnumber, n);
		//pos+=min(charnumber,fsize-pointer);
		printf("%x,%d",*(resultbuf+pos-1),pointer_resbuf);
		if(getBCC2(resultbuf+pos,pointer_resbuf-1)==FLAG){
		resultbuf[pointer_resbuf+3] = ESCAPE0;
		resultbuf[pointer_resbuf+4] = ESCAPE1;
		resultbuf[pointer_resbuf+5] = FLAG;
		else if(getBCC2(resultbuf+pos,pointer_resbuf-1)==ESCAPE1){
		resultbuf[pointer_resbuf+3] = ESCAPE1;
		resultbuf[pointer_resbuf+4] = ESCAPE2;
		resultbuf[pointer_resbuf+5] = FLAG;
		}
		else {
		resultbuf[pointer_resbuf+3] = getBCC2(resultbuf+pos,pointer_resbuf-1);
		resultbuf[pointer_resbuf+4] = FLAG;
		}
		}
*/

	newpointer = datapacket(buffer, resultbuf+pos, pointer, charnumber, n);

	//pos+=min(charnumber,fsize-pointer);

	resultbuf[pointer_resbuf+4] = FLAG;
	int i;


	printf("\n\nData: %x",resultbuf[0]);
	for(i=1;i<pointer_resbuf+5;i++){
		printf(" %x",(unsigned char)resultbuf[i]);
	}
	printf("\n");

	llwrite(fd,resultbuf,pointer_resbuf+5);

	return newpointer-pointer;
}

int writeDISC(int fd){
	int res;
	char buffer[5];
	buffer[0] = FLAG;
	buffer[1] = A;
	buffer[2] = CDISC;
	buffer[3] = A^CDISC; //BCC
	buffer[4] = FLAG;

	res = llwrite(fd,buffer,5);
	if(res>0)
		printf("Comando enviado (DISC)\n");
	else
		exit(1);
	return res;
}

int writeUA(int fd){
	int res;
	char buffer[5];
	buffer[0] = FLAG;
	buffer[1] = A2;
	buffer[2] = CUA;
	buffer[3] = A2^CUA; //BCC
	buffer[4] = FLAG;

	res = llwrite(fd,buffer,5);
	if(res>0)
		printf("Resposta enviada (UA)\n");
	else
		exit(1);
	return res;
}

int checkRR(char* buffer, char ns){
	printf("%x,%x,%x,%x,%x\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
	if(	 buffer[0]==FLAG
	  && buffer[1]==A
	  && buffer[4]==FLAG)
		if(ns==0){
			if(buffer[2]==0x25 && buffer[3]==A^0x25)
				return TRUE;
		}
		else{
			if(buffer[2]==0x5 && buffer[3]==A^0x5)
				return TRUE;
		}
	else
		return FALSE;
}

void read5bytes(int fd, char* buffer){
	int count=0,i=0;
	do{
		i=llread(fd,buffer,i);
		count++;
	}while(count<=10 && i<5);
}

int checkUA(char* buffer){
	if(	 buffer[0]==FLAG
	  && buffer[1]==A
	  && buffer[2]==CUA
	  && buffer[3]==(A^CUA)
	  && buffer[4]==FLAG)
		return TRUE;
	else
		return FALSE;
}

int checkI(char* buffer){
	printf("%x,%x,%x,%x,%x\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
	if(	 buffer[0]==FLAG
	  && buffer[1]==A2
	  && buffer[2]==CDISC
	  && buffer[3]==(A2^CDISC)
	  && buffer[4]==FLAG)
		return TRUE;
	else
		return FALSE;
}

int checkDISC(char* buffer){
	printf("%x,%x,%x,%x,%x\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4]);
	if(	 buffer[0]==FLAG
	  && buffer[1]==A2
	  && buffer[2]==CDISC
	  && buffer[3]==(A2^CDISC)
	  && buffer[4]==FLAG)
		return TRUE;
	else
		return FALSE;
}

int datapacket(char *file, unsigned char *datapacket, int y, int k, char n)
{
	int i=0, j, old_k=k, *slength;
	char L2, L1, bcc2=0;
	char sbuffer[k*2];
	k += stuff(file+y,k,sbuffer,slength);

	L2 = old_k>>8;
	L1 = old_k;

	bcc2 = n^L2^L1;

	for(i=0;i<old_k;i++){
		bcc2=bcc2^file[y+i];
		printf(" %x ", file[y+i]);
	}

	printf("\nBCC2: %x\n", bcc2);
	i=0;

	datapacket[i++]=0x00;
	if(n==FLAG){
		datapacket[i++]=ESCAPE0;
		datapacket[i++]=ESCAPE1;
	}
	else if(n==ESCAPE0){
		datapacket[i++]=ESCAPE0;
		datapacket[i++]=ESCAPE2;
	}
	else
		datapacket[i++]=n;

	if(L2==FLAG){
		datapacket[i++]=ESCAPE0;
		datapacket[i++]=ESCAPE1;
	}
	else if(L2==ESCAPE0){
		datapacket[i++]=ESCAPE0;
		datapacket[i++]=ESCAPE2;
	}
	else
		datapacket[i++]=L2;

	if(L1==FLAG){
			datapacket[i++]=ESCAPE0;
		datapacket[i++]=ESCAPE1;
	}
	else if(L1==ESCAPE0){
		datapacket[i++]=ESCAPE0;
		datapacket[i++]=ESCAPE2;
	}
	else
		datapacket[i++]=L1;

	for(j=0;j<k;j++){
		datapacket[i]=sbuffer[j];
		i++;
	}

	if (bcc2 == FLAG)
	{
		datapacket[i++] = ESCAPE0;
		datapacket[i++] = ESCAPE1;
	}
	else if(bcc2  == ESCAPE0)
	{
		datapacket[i++] = ESCAPE0;
		datapacket[i++] = ESCAPE2;
	}
	else
		datapacket[i++]=bcc2;

	pointer_resbuf = i;

	printf("K: %d\n",old_k);
	return y+old_k;
}

int controlpacket(char* controlpacket, char cc, char* sizeoffile, char* filename){ //CHECK

	int count=3, i;
	int sizeoffilename=strlen(filename);
	int sizeofsizeoffile=strlen(sizeoffile);

	controlpacket[0]=cc;
	controlpacket[1]=0;
	controlpacket[2]=sizeofsizeoffile;

	for(i=0;i<sizeofsizeoffile;i++){
		controlpacket[count+i]=sizeoffile[i];
	}

	count+=i;
	controlpacket[count]=1;
	controlpacket[++count]=sizeoffilename;
	count++;
	for(i=0;i<sizeoffilename;i++){
		controlpacket[count+i]=filename[i];
	}
	count+=i;
	return count;
}

int min(int a, int b){
	if(a<b) return a;
	else
		return b;
}

int updateSequenceNumber(char n){
	if(n==255) return 0;
	else
		return n+1;
}

int main(int argc, char** argv)
{
	int fd;

	fd=llopen(0);

	transfDados(fd,argv[1],argv[2]);

	llclose(fd);

	return 0;
}
