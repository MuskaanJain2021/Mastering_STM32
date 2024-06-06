/*Bit-by-Bit Reversal
When you reverse the bits of a 32-bit integer bit by bit, you swap the first bit with the 32nd bit, the second bit with the 31st bit, and so on. 
This operation completely reverses the order of all bits in the integer.

For example:

Original (binary): 11011110101011011011111011101111 (0xDEADBEEF)
Reversed (binary): 11110111101110111101011010111101 (0xF77DB57B)
hint : array of structure func ptr for switch case
*/
#include <stdio.h>
#include <stdint.h>

#define MAX_BITS 32 // Maximum number of bits supported

// Define macros for word sizes using the size of the types
#define SIZE_OF_POINTER(ptr) ((size_t)((char *)((ptr) + 1) - (char *)(ptr)))


typedef void (*DecToBinFunc)(void* num, char* bin);
typedef void* (*ReverseBitsFunc)(void* num);

// Function to convert a decimal number to binary string (static allocation)
void DecToBin8(void* num, char* bin) {
    uint8_t n = *((uint8_t*)num);
    for (int idx = 7; idx >= 0; idx--) {
        bin[7 - idx] = (n & (1 << idx)) ? '1' : '0';
    }
    bin[8] = '\0';
}

void DecToBin16(void* num, char* bin) {
    uint16_t n = *((uint16_t*)num);
    for (int idx = 15; idx >= 0; idx--) {
        bin[15 - idx] = (n & (1 << idx)) ? '1' : '0';
    }
    bin[16] = '\0';
}

void DecToBin32(void* num, char* bin) {
    uint32_t n = *((uint32_t*)num);
    for (int idx = 31; idx >= 0; idx--) {
        bin[31 - idx] = (n & (1 << idx)) ? '1' : '0';
    }
    bin[32] = '\0';
}

// Helper function to reverse bits of a number
uint32_t reverseBitsHelper(uint32_t num, size_t bits) {
    uint32_t reversed = 0;
    for (int i = 0; i < bits; i++) {
        reversed |= (num & 1) << (bits - 1 - i);
        num >>= 1;
    }
    return reversed;
}

// Functions to reverse bits for different sizes
uint8_t reverseBits8(void* num) {
    uint8_t n = *((uint8_t*)num);
    return (uint8_t)reverseBitsHelper(n, 8);
}

uint16_t reverseBits16(void* num) {
    uint16_t n = *((uint16_t*)num);
    return (uint16_t)reverseBitsHelper(n, 16);
}

uint32_t reverseBits32(void* num) {
    uint32_t n = *((uint32_t*)num);
    return reverseBitsHelper(n, 32);
}

typedef struct {
    size_t size;
    DecToBinFunc decToBin;
    ReverseBitsFunc reverseBits;
} TypeHandler;

uint8_t sample8;
uint16_t sample16;
uint32_t sample32;

// Array of type handlers
TypeHandler typeHandlers[] = {
    {SIZE_OF_POINTER(&sample8), DecToBin8, (ReverseBitsFunc)reverseBits8},
    {SIZE_OF_POINTER(&sample16), DecToBin16, (ReverseBitsFunc)reverseBits16},
    {SIZE_OF_POINTER(&sample32), DecToBin32, (ReverseBitsFunc)reverseBits32}
};

int main() {
    // Static allocation for binary string representation
    char binStr[MAX_BITS + 1]; // +1 for the null terminator

    // Example with 32-bit number (WORD)
    uint32_t x = 0xDEADBEEF;
    typeHandlers[2].decToBin(&x, binStr); // 32-bit handler
    printf("Original value in binary: %s\n", binStr);
    printf("Original value in hex: 0x%X\n", x);

    uint32_t reversed32 = (uint32_t)(typeHandlers[2].reverseBits)(&x);
    typeHandlers[2].decToBin(&reversed32, binStr); // 32-bit handler
    printf("Reversed value by bit by bit swapping technique in binary: %s\n", binStr);
    printf("Reversed value in hex: 0x%X\n", reversed32);

    // Example with 16-bit number (HALF_WORD)
    uint16_t y = 0xABCD;
    typeHandlers[1].decToBin(&y, binStr); // 16-bit handler
    printf("Original value (16-bit) in binary: %s\n", binStr);
    printf("Original value (16-bit) in hex: 0x%X\n", y);

    uint16_t reversed16 = (uint16_t)(typeHandlers[1].reverseBits)(&y);
    typeHandlers[1].decToBin(&reversed16, binStr); // 16-bit handler
    printf("Reversed value (16-bit) by bit by bit swapping technique in binary: %s\n", binStr);
    printf("Reversed value (16-bit) in hex: 0x%X\n", reversed16);

    // Example with 8-bit number (BYTE)
    uint8_t z = 0xAB;
    typeHandlers[0].decToBin(&z, binStr); // 8-bit handler
    printf("Original value (8-bit) in binary: %s\n", binStr);
    printf("Original value (8-bit) in hex: 0x%X\n", z);

    uint8_t reversed8 = (uint8_t)(typeHandlers[0].reverseBits)(&z);
    typeHandlers[0].decToBin(&reversed8, binStr); // 8-bit handler
    printf("Reversed value (8-bit) by bit by bit swapping technique in binary: %s\n", binStr);
    printf("Reversed value (8-bit) in hex: 0x%X\n", reversed8);

    return 0;
}
