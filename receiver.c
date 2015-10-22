/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>

#define BAUDRATE B38400
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1
#define FLAG 0x7E
#define START 0
#define F 1
#define FA 2
#define FAC 3
#define A 0x03
#define A2 0x01
#define CSET 0x03
#define CS0 0x00
#define CS1 0x02
#define CUA 0x07
#define CDISC 0x0B
#define SUCCESS 4
#define ESCAPE0 0X7d
#define ESCAPE1 0x5e
#define ESCAPE2 0x5d

volatile int STOP=FALSE;
volatile int STOP2=FALSE;
volatile int I=FALSE;
volatile int ENDOFDATA=FALSE;
struct termios oldtio,newtio;
char buf[255],buf2[5],databuf[255];
unsigned char filename[50];

int lerdados(unsigned char* buffer,unsigned char* dados,  int pointer);

int llopen(int porta){
	int fd;
	char nomeporta[11];
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

	//fcntl(fd,F_SETFL, FNDELAY);
	printf("Serial port open\n\n");
	return fd;
}

int llread(int fd, unsigned char* buffer)
{
	int res,i=0;
	char state=START;
	while(state!=SUCCESS)
	{
		res = read(fd,buffer+i,sizeof(char));
		switch(state){
			case START:
			if(buffer[i]==FLAG){
				state = F;
				i++;
			}
			break;

			case F:
			if(buffer[i]==FLAG){
				state = F;
			}
			else{
				i++;
				state=FA;
			}
			break;

			case FA:
			if(buffer[i] == F){
				state = F;
				i=1;
			}
			else{
				state =FAC;
				i++;
			}
			break;

			case FAC:
			do{
				i++;
				res = read(fd,buffer+i,1);
			}while(buffer[i]!=FLAG);
			state=SUCCESS;
			break;
		}
	}
	state=START;
	return i;
}

int llwrite(int fd, char* buffer, int length){
	int res;
	res = write(fd,buffer,length);
	return res;
}

void llclose(int fd){
	sleep(1);
	tcsetattr(fd,TCSANOW,&oldtio);
	close(fd);
	printf("\nSerial port closed, exiting...\n");
}

//Verifica se o SET foi bem recebido, devolvendo TRUE ou FALSE
int checkSET(char* buffer){
	if(	 buffer[0]!=FLAG
	  || buffer[1]!=A
	  || buffer[2]!=CSET
	  || buffer[3]!=(buf[1]^buf[2])
	  || buffer[4]!=FLAG)
		return FALSE;
	else
		return TRUE;
}

//Devolve TRUE quando a primeira trama I é detectada
int checkFirstI(char* buffer){
	if(	 buffer[0]==FLAG
	  && buffer[1]==A
	  && (buffer[2]==CS0 || buffer[2]==CS1))
		return TRUE;
	else
		return FALSE;
}

//Escreve o UA de resposta para o SET, devolvendo o numero de bytes escritos
int writeUA(int fd){
	char buffer[5];
	int res;
	buffer[0]=FLAG;
	buffer[1]=A;
	buffer[2]=CUA;
	buffer[3]=A^CUA;
	buffer[4]=FLAG;

	res = llwrite(fd,buffer,5);
	if(res>0)
		printf("Resposta enviada (UA)\n");
	else
		exit(1);
	return res;
}

//Retira do buffer apenas o que e informacao e coloca no data, devolvendo um pointer para a posicao actual no data
int getData(char* buffer, char* data){

	int k,i=4;
	for(k=0;buffer[i]!=FLAG;k++){
		data[k]=buffer[i];
		if(data[k-1]==ESCAPE0 && data[k]==ESCAPE1){
			data[k-1]=FLAG;
			k--;
		}
		else if(data[k-1]==ESCAPE0 && data[k]==ESCAPE2){
			data[k-1]=ESCAPE0;
			k--;
		}
		i++;
	}
	return k-1;
}

//Calcula o BCC2 e verifica se é igual ao que foi recebido, devolvendo TRUE ou FALSE
int checkData(char* buffer, int pos)
{
	char temp=0;
	int k;
	for(k=0;k<pos;k++){
		temp = temp^buffer[k];
	}
	if(temp==buffer[pos])
	{
		return TRUE;
	}
	else
		return FALSE;
}

//Verifica se o I foi bem recebido
int checkI(char* buffer,int cns){
	int res,i;
	char data[15000];
	if(	 buffer[0]!=FLAG
	  || buffer[1]!=A
	  || buffer[2]!=cns
	  || buffer[3]!=(A^cns))
	{
		return FALSE;
	}
	else
	{
		res = getData(buffer,data);
		res = checkData(data,res);
		return res;
	}
}

//Verifica se o DISC foi bem recebido, devolvendo TRUE ou FALSE
int checkDISC(char* buffer){
	if(	 buffer[0]==FLAG
	  && buffer[1]==A
	  && buffer[2]==CDISC
	  && buffer[3]==(A^CDISC)
	  && buffer[4]==FLAG){
		return TRUE;
	}
	else{
		return FALSE;
	}
}

//Escreve o RR de resposta ao I, devolvendo o numero de bytes escritos
int writeRR(int fd, int cnr){
	char buffer[5];
	int res;
	buffer[0]=FLAG;
	buffer[1]=A;
	buffer[2]=cnr;
	buffer[3]=A^cnr;
	buffer[4]=FLAG;

	res = llwrite(fd,buffer,5);
	if(res>0)
		printf("Resposta enviada (RR)\n");
	else
		exit(1);
	return res;
}

//Verifica se o ultimo UA foi bem recebido, devolvendo TRUE ou FALSE
int checkUA(char* buffer){
	if(	 buffer[0]==FLAG
	  && buffer[1]==A2
	  && buffer[2]==CUA
	  && buffer[3]==(A2^CUA)
	  && buffer[4]==FLAG)
		return TRUE;
	else
		return FALSE;
}

//Escreve o DISC de resposta, devolvendo o numero de bytes escritos
int writeDISC(int fd){
	char buffer[5];
	int res;
	buffer[0]=FLAG;
	buffer[1]=A2;
	buffer[2]=CDISC;
	buffer[3]=(A2^CDISC);
	buffer[4]=FLAG;

	res = llwrite(fd,buffer,5);
	if(res>0)
		printf("Comando enviado (DISC)\n");
	else
		exit(1);
	return res;
}

//Recebe o SET e responde UA
void estabelecimento(int fd, unsigned char* buffer)
{
	int res;
	int isValid = FALSE, next = FALSE, canProceed = FALSE;
	while(canProceed != TRUE || next!=TRUE)
	{
		res = llread(fd,buffer);
		isValid = checkSET(buffer);
		if(isValid==TRUE)
		{
			printf("Comando recebido (SET)\n");
			canProceed=TRUE;
			writeUA(fd);
		}
		else
		{
			next = checkFirstI(buffer);
		}
	}
}

//Recebe as tramas de dados e responde RR
int transfDados(int fd, char* buffer, char* data, int filesize)
{
	char dados[15000];
	int res=0,i, a=0;
	int pointer=0;
	char nr,cns,cnr;
	nr=0x0;
	cns=0x0;
	cnr=0x5;
	int isValid = FALSE, next = FALSE;

	while(next!=TRUE)
	{
		isValid = checkI(buffer,cns);

		if(isValid==TRUE)
		{
			getData(buffer,data);
			if (data[0]==1)
			{
				printf("Recebida trama de controlo de dados (start)\n");
				char fsize[data[2]];
				while(a<data[2])
				{
					fsize[a]=data[a+3];
					a++;
				}
				filesize = atoi(fsize);
				a=0;
				while(a<=data[4+data[2]])
				{
					filename[a]=data[4+data[2]+a];
					//printf("%c",filename[a]);
					a++;
				}
				filename[a]='\0';
			}
			else if(data[0]==0)
			{
				pointer = lerdados(data,dados,pointer);
				printf("A escrever dados na posicao %d de um total de %d\n",pointer,filesize);
			}
			else if(data[0]==2)
			{
				printf("Recebida trama de controlo de dados (end)\n");
				/*if(filesize!=(data[3]))
				{
					printf("Dados corrompidos, tamanho do ficheiro nao e valido.%d\n",data[3]);
					exit(1);
				}
				if(filename!=data[6])
				{
					printf("Dados corrompidos. Impossivel terminar.\n");
					exit(1);
				}*/
			}
			else
			{
				printf("Trama de dados corrompida.\n");
			}
			//Verifica o valor do N(r) anterior para o poder alterar
			if(nr==0x1){
				nr=0x0;
				cns=0x0;
				cnr=0x5;
			}
			else{
				nr=0x1;
				cns=0x2;
				cnr=0x25;
			}
			writeRR(fd,cnr);//Escreve o RR de resposta que permite ao Sender identificar se enviou o I bem ou necessita de enviar outra vez
		}
		else
		{

			next = checkDISC(buffer);
			if(next==FALSE)
			{
				writeRR(fd,cnr);
			}
			else{
				for(i=0; i<filesize; i++){
					data[i] = dados[i];
				}
				return filesize;
			}
		}
		llread(fd,buffer);
	}
}

//Recebe o DISC, responde DISC e recebe o UA
void terminacao(fd){
	char* buffer[255];

	char next = FALSE, isValid=TRUE;
	while(next!=TRUE){
		if(isValid==TRUE){
			printf("Comando recebido (DISC)\n");
			writeDISC(fd);
		}
		next = checkUA(buffer);
		if(next!=TRUE){
			llread(fd,buffer);
			isValid = checkDISC(buffer);
		}
	}
	printf("Comando recebido (UA)\n");
}

int main(int argc, char** argv)
{
	FILE *file;
	char data[150000];
	int fd, i;
	fd = llopen(0);
	int filesize;

	unsigned char firstData[10000];

	estabelecimento(fd, firstData);

	filesize = transfDados(fd, firstData, data, filesize);
	terminacao(fd);
	/*printf("filename: \n");
	for(i=0; i<strlen(filename); i++){
		printf("%c",filename[i]);
	}

	printf("Dados:\n ");
	for(i=0;i<filesize;i++){
		printf("%x ",data[i]);
	}*/
	file=open(filename+1, O_WRONLY | O_CREAT, S_IWRITE | S_IREAD);
	write(file,data,filesize);
	close(file);

	llclose(fd);
	return 0;
}

// cenas ---------------------------------------------------------------------------

//Le os pacotes de dados e separa o que sao  dados do que sao os campos c, n , L1 e L2
//Devolve um pointer indicando o sitio nos dados em que esta funcao terminou
int lerdados(unsigned char* buffer,unsigned char* dados,  int pointer)
{
	int k, a;
	k=256*buffer[2]+buffer[3]; //Calcula o numero total de octetos que vão ser enviados no campo dos dados
	printf("k: %d\n", k);
	a=4;
	while(k>0)
	{
		dados[pointer]=buffer[a];
		a++;
		k--;
		pointer++;
	}
	return pointer;
}

