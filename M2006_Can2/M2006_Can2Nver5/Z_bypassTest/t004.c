#include <stdio.h>
#include <string.h>

char rx_arrays[4][5] = {"+1122","-3344","+5566","-7788"};
char rx_up[5];
char rx_left[5];
char rx_down[5];
char rx_right[5];
int rx_getNumValue[4] = {1,1,1,1};

int getRxNumValue(char array[],int para_getNumValue){
	int reNum = para_getNumValue;
	for(int i = 0 ,j = 1000;i <= 3;i++){
		
		
		reNum +=  (array[i+1] -48) * j;
		printf("arr = %c\n",array[i]);
		printf("reNum = %d\n",reNum);
		j /= 10;
	}
	return reNum -1;

}
 
int main()
{
    char str[10] = "123456";
    int num = 0;
 
    const char* ptr = str;
    while(*ptr)
    {
        num *= 10;
        num += *ptr - '0';
        ptr++;
    }
    printf("num = %d\n",num);
    
    num = -3;
    printf("num = %d\n",num%5);
    
    for(int i = 0; i < 4; i++){
    	printf("rx_getNumValue = %d\n",rx_getNumValue[i]);
    	for(int w = 0; w < 4; w++){
    		printf("STR = %s\n",rx_arrays[i]);
		}
    	
    	rx_getNumValue[i] = getRxNumValue(rx_arrays[i], rx_getNumValue[i]);
    	printf("rx_getNumValue = %d\n",rx_getNumValue[i]);
    	/*
			if(rx_getNumValue[i] > 2000){
				printf("num = %d\n",num%5);
			}else if(rx_getNumValue[i] <- 2000){
				HAL_UART_Transmit(&huart1, (uint8_t *)"-", sizeof("-"), 10);
			}else{
				HAL_UART_Transmit(&huart1, (uint8_t *)"*", sizeof("*"), 10);
			}
		*/
		}
    
    return 0;
}
