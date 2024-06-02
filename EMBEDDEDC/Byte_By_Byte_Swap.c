#include <stdio.h>
#include <stdint.h>


//Divide 32 bit integer into half and then further half .Reverse the order of halves.

/*
 UPPER     LOWER   HALVES  FINAL RESULT
 0xDEAD   0XBEEF  ----->  0XEFBEADDE
 0XBE|0XEF|0XDE|0XAD---->SWAP UPPER AND LOWER HALF
 
 0XEF|OXBE|0XAD|0XDE  --->MIRROR ABOVE RESULT/REVERSE THE ORDER OF HALVES


*/


uint32_t swap(uint32_t org_num)
{
	
    uint32_t  h_nibble = 0;
	uint32_t  l_nibble = 0;
	
    h_nibble = org_num >> 16; //0xDEAD
	l_nibble=  org_num & 0xFFFF; //0xBEEF
	
	//0xDEAD =>  0XADDE
	h_nibble =  (h_nibble >> 8) | (h_nibble & 0xFF00) << 8;
	//0XBEEF =>  0XEFBE
	l_nibble =  (l_nibble >> 8) | (l_nibble & 0xFF00) << 8;
	 
    uint32_t coverted_num = (l_nibble <<16) | h_nibble;//0XEFBEADDE
	
	
}

uint32_t convertLittleToBigEndian(uint32_t orig_hex) {
    uint32_t ret = 0;
    while (orig_hex != 0) {
        ret <<= 8;
        ret |= orig_hex & 0xFF;
        orig_hex >>= 8;
    }
    return ret;
}

uint32_t reverseBytesApproach2(uint32_t num) {
    unsigned char bytes[4];
    bytes[0] = num & 0xFF;
    bytes[1] = (num >> 8) & 0xFF;
    bytes[2] = (num >> 16) & 0xFF;
    bytes[3] = (num >> 24) & 0xFF;
    return (bytes[0] << 24) | (bytes[1] << 16) | (bytes[2] << 8) | bytes[3];
}



int main() {
    uint32_t x = 0xDEADBEEF;
    unsigned char *c = (unsigned char *)&x;
    
    if (*c == 0xEF)
        printf("Little endian\n");
    else
        printf("Big endian\n");
    
    printf("Original value: 0x%x\n", x);
	
    // Convert little-endian to big-endian
    uint32_t bigEndian = convertLittleToBigEndian(x);
    printf("After conversion to big-endian: 0x%x\n", bigEndian);
	
	// Divide the 32-bit integer into half and then further half
	uint32_t Reverse_hex = swap(x);
	printf("After conversion to big-endian: 0x%x\n",Reverse_hex);
	
	//Convert the integer into a 4-character array and swap the characters
	uint32_t Rev_Array = reverseBytesApproach2(x);
	printf("After conversion to big-endian: 0x%x\n",Rev_Array);
	
	
	
	return 0;
}
	
	
	
	
	
  
