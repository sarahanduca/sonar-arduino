#define F_CPU 16000000UL      //define a frequencia do microcontrolador - 16MHz

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>

#define set_bit(y, bit) (y|=(1<<bit))		//COLOCA EM 1 O BIT x DA VARIÁVEL y
#define clr_bit(y, bit) (y&=~(1<<bit))		//COLOCA EM 0 O BIT x DA VARIÁVEL y
#define cpl_bit(y, bit) (y^=(1<<bit))		//TROCA O ESTADO LÓGICO DO BIT x DA VARIAVEL y
#define tst_bit(y, bit) (y&(1<<bit))		//RETORNA 0 OU 1 CONFORME LEITURA DO BIT

#define DADOS_LCD		PORTD //4 bits de dados do LCD no PORTD
#define nibble_dados	1

#define CONTR_LCD 		PORTB //PORT com os pinos de controle do LCD (pino R/W em 0)
#define E 				PB3     //PINO DE HABILITACAO DO LCD
#define RS 				PB2	  //PINO PARA INFORMAR SE O DADO EH UMA INSTRUCAO OU CARACTERE

#define tam_vetor 		5 	  //numero de digitos individuais para a conversao por ident_num()
#define conv_ascii 		48    //se ident_num() deve retornar um numero no formato ASCII (0 para formato normal)

//sinal de habilitacao do LCD
#define pulso_enable() _delay_us(1); set_bit(CONTR_LCD, E); _delay_us(1); clr_bit(CONTR_LCD, E); _delay_us(45)

#define DISPARO PB2
unsigned int Inicio_Sinal, Distancia;

//prog_char mensagem1[] = "DISTANCIA = cm\0";	//mensagem armazenada na memória flash
//prog_char mensagem2[] = "xxx\0";
char *c = "xxx\0";

ISR(TIMER1_CAPT_vect){
  cpl_bit(TCCR1B, ICES1);
  if(!tst_bit(TCCR1B, ICES1)) Inicio_Sinal = ICR1;
  else Distancia = (ICR1 - Inicio_Sinal)/58;
}

void cmd_LCD(unsigned char c, char cd)
{
	if(cd == 0)		//INSTRUCAO
	clr_bit(CONTR_LCD, RS);
	else			//CARACTER
	set_bit(CONTR_LCD, RS);
	//PRIMEIRO NIBBLE DE DADOS
	#if(nibble_dados)
	DADOS_LCD = (DADOS_LCD & 0x0F)|(0xF0 & c);  //COMPILA O CÓDIGO PARA OS PINOS DE DADOS DO LCD NOS 4 MSB DO PORT
	#else
	DADOS_LCD = (DADOS_LCD & 0xF0)|(c>>4);		//COMPILA O CÓDIGO PARA OS PINOS DE DADOS DO LCD NOS 4 LSB DO PORT
	#endif

	pulso_enable();


	//SEGUNDO NIBBLE DE DADOS
	#if(nibble_dados)
	DADOS_LCD = (DADOS_LCD & 0x0F)|(0xF0 & (c<<4));	//COMPILA O CÓDIGO PARA OS PINOS DE DADOS DO LCD NOS 4 MSB DO PORT
	#else
	DADOS_LCD = (DADOS_LCD & 0xF0)|(0x0F & c);		//COMPILA O CÓDIGO PARA OS PINOS DE DADOS DO LCD NOS 4 LSB DO PORT
	#endif

	pulso_enable();

	if((cd==0) && (c<4)) //SE FOR INSTRUCAO DE RETORNO OU LIMPEZA ESPERA LCD ESTAR PRONTO
	_delay_ms(2);
}



void inic_LCD_4bits() //Sequência feita pelo fabricante
{
	clr_bit(CONTR_LCD, RS); //RS EM 0 INDICA QUE O DADO PARA O LCD SERÁ UMA INSTRUCAO
	clr_bit(CONTR_LCD, E);  //PINO DE HABILITACAO EM 0

	_delay_ms(20);

	#if (nibble_dados)
	DADOS_LCD = (DADOS_LCD & 0x0F) | 0x30;
	#else
	DADOS_LCD = (DADOS_LCD & 0xF0) | 0x03;
	#endif

	pulso_enable();
	_delay_ms(5);
	pulso_enable();
	_delay_ms(200);
	pulso_enable();

	#if (nibble_dados)
	DADOS_LCD = (DADOS_LCD & 0x0F) | 0x20;
	#else
	DADOS_LCD = (DADOS_LCD & 0xF0) | 0x02;
	#endif

	pulso_enable();
	cmd_LCD(0x28,0);
	cmd_LCD(0x08,0);
	cmd_LCD(0x01,0);
	cmd_LCD(0x0C,0);
	cmd_LCD(0x80,0);
}


void escreve_LCD(char *c)
{
	for (; *c!=0;c++) cmd_LCD(*c,1);
}

void escreve_LCD_FLASH(const char *c)
{
  for(;pgm_read_byte(&(*c))!=0;c++) cmd_LCD(pgm_read_byte(&(*c)),1);
}

void ident_num(unsigned int valor, unsigned char *disp)
{
  unsigned char n;
  
  for(n=0; n<tam_vetor; n++)
    disp[n] = 0 + conv_ascii;
  
  do
  {
    *disp = (valor%10) + conv_ascii;
    valor /= 10;
    disp++;
  }while(valor != 0 );

}


int main(void)
{
  unsigned char digitos[tam_vetor];
	DDRB = 0b00000010; //PORTB COMO SAÍDA
	DDRD = 0xFF;
  PORTB = 0b11111101;

  TCCR1B = (1 << ICES1) | (1 << CS11);
  TIMSK1 = 1 << ICIE1;
  sei();


	inic_LCD_4bits();
	escreve_LCD("DISTANCIA =   cm\0"); //string armazenada na RAM

    //escreve_LCD_FLASH(mensagem);
  	escreve_LCD(*c);
  
  	while(1){
      set_bit(PORTB, DISPARO);
      _delay_us(10);
      clr_bit(PORTB, DISPARO);
  	  cmd_LCD(0xC0,0); //desloca cursor
      if(Distancia < 431){
        ident_num(Distancia, digitos);
        cmd_LCD(digitos[2], 1);
        cmd_LCD(digitos[1], 1);
        cmd_LCD(digitos[0], 1);
      }
      else escreve_LCD(c);
      _delay_ms(50);

    }
}