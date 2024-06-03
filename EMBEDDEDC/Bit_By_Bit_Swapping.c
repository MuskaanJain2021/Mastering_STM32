/*Bit-by-Bit Reversal
When you reverse the bits of a 32-bit integer bit by bit, you swap the first bit with the 32nd bit, the second bit with the 31st bit, and so on. 
This operation completely reverses the order of all bits in the integer.

For example:

Original (binary): 11011110101011011011111011101111 (0xDEADBEEF)
Reversed (binary): 11110111101110111101011010111101 (0xF77DB57B)

*/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#define SIZE_OF(var) ((char*)(&var + 1) - (char*)(&var))

// Function to convert a decimal number to binary string
char* DecToBin(void* num, size_t size) {
    int bit_count = size * 8; // Calculate the number of bits
    char* bin = (char*)malloc(bit_count + 1); // Allocate memory for bits + null terminator
    if (bin == NULL) {
        fprintf(stderr, "Memory allocation failed in DecToBin function\n");
        return NULL; // Check if memory allocation was successful
    }
    bin[bit_count] = '\0'; // Set the null terminator at the end of the string
    uint32_t n = 0;
    if (size == SIZE_OF(*(uint8_t*)num)) {
        n = *((uint8_t*)num);
    } else if (size == SIZE_OF(*(uint16_t*)num)) {
        n = *((uint16_t*)num);
    } else if (size == SIZE_OF(*(uint32_t*)num)) {
        n = *((uint32_t*)num);
    }

    for (int idx = bit_count - 1; idx >= 0; idx--) {
        if (n & (1 << idx)) // Check if the bit is set
            bin[bit_count - 1 - idx] = '1'; // Set the corresponding index in the string to '1'
        else
            bin[bit_count - 1 - idx] = '0'; // Set the corresponding index in the string to '0'
    }
    return bin;
}

// Function to reverse bits of a number
void* reverseBits(void* num, size_t size) {
    uint32_t n = 0;
    if (size == SIZE_OF(*(uint8_t*)num)) {
        n = *((uint8_t*)num);
    } else if (size == SIZE_OF(*(uint16_t*)num)) {
        n = *((uint16_t*)num);
    } else if (size == SIZE_OF(*(uint32_t*)num)) {
        n = *((uint32_t*)num);
    }

    uint32_t reversed = 0;
    int bit_count = size * 8;
    for (int i = 0; i < bit_count; i++) {
        reversed |= (n & 1) << (bit_count - 1 - i);
        n >>= 1;
    }

    if (size == SIZE_OF(*(uint8_t*)num)) {
        uint8_t* result = (uint8_t*)malloc(sizeof(uint8_t));
        *result = (uint8_t)reversed;
        return result;
    } else if (size == SIZE_OF(*(uint16_t*)num)) {
        uint16_t* result = (uint16_t*)malloc(sizeof(uint16_t));
        *result = (uint16_t)reversed;
        return result;
    } else if (size == SIZE_OF(*(uint32_t*)num)) {
        uint32_t* result = (uint32_t*)malloc(sizeof(uint32_t));
        *result = (uint32_t)reversed;
        return result;
    }
    return NULL; // In case the size doesn't match any expected types
}

int main() {
    // Example with 32-bit number
    uint32_t x = 0xDEADBEEF;
    // Convert number to binary string
    char* binStr = DecToBin(&x, SIZE_OF(x));
    if (binStr == NULL) {
        fprintf(stderr, "Failed to convert number to binary string in main function\n");
        return 1;
    }
    printf("Original value in binary: %s\n", binStr);
    printf("Original value in hex: 0x%X\n", x);

    uint32_t* reversed = (uint32_t*)reverseBits(&x, SIZE_OF(x));
    if (reversed == NULL) {
        fprintf(stderr, "Failed to reverse bits in main function\n");
        free(binStr);
        return 1;
    }
    // Convert reversed number to binary string
    char* RevBinStr = DecToBin(reversed, SIZE_OF(*reversed));
    if (RevBinStr == NULL) {
        fprintf(stderr, "Failed to convert reversed number to binary string in main function\n");
        free(binStr);
        free(reversed);
        return 1;
    }
    printf("Reversed value by bit by bit swapping technique in binary: %s\n", RevBinStr);
    printf("Reversed value in hex: 0x%X\n", *reversed);

    // Free allocated memory
    free(binStr);
    free(RevBinStr);
    free(reversed);

    // Example with 16-bit number
    uint16_t y = 0xABCD;
    binStr = DecToBin(&y, SIZE_OF(y));
    if (binStr == NULL) {
        fprintf(stderr, "Failed to convert number to binary string in main function\n");
        return 1;
    }
    printf("Original value (16-bit) in binary: %s\n", binStr);
    printf("Original value (16-bit) in hex: 0x%X\n", y);

    uint16_t* reversed16 = (uint16_t*)reverseBits(&y, SIZE_OF(y));
    if (reversed16 == NULL) {
        fprintf(stderr, "Failed to reverse bits in main function\n");
        free(binStr);
        return 1;
    }
    RevBinStr = DecToBin(reversed16, SIZE_OF(*reversed16));
    if (RevBinStr == NULL) {
        fprintf(stderr, "Failed to convert reversed number to binary string in main function\n");
        free(binStr);
        free(reversed16);
        return 1;
    }
    printf("Reversed value (16-bit) by bit by bit swapping technique in binary: %s\n", RevBinStr);
    printf("Reversed value (16-bit) in hex: 0x%X\n", *reversed16);

    // Free allocated memory
    free(binStr);
    free(RevBinStr);
    free(reversed16);

    // Example with 8-bit number
    uint8_t z = 0xAB;
    binStr = DecToBin(&z, SIZE_OF(z));
    if (binStr == NULL) {
        fprintf(stderr, "Failed to convert number to binary string in main function\n");
        return 1;
    }
    printf("Original value (8-bit) in binary: %s\n", binStr);
    printf("Original value (8-bit) in hex: 0x%X\n", z);

    uint8_t* reversed8 = (uint8_t*)reverseBits(&z, SIZE_OF(z));
    if (reversed8 == NULL) {
        fprintf(stderr, "Failed to reverse bits in main function\n");
        free(binStr);
        return 1;
    }
    RevBinStr = DecToBin(reversed8, SIZE_OF(*reversed8));
    if (RevBinStr == NULL) {
        fprintf(stderr, "Failed to convert reversed number to binary string in main function\n");
        free(binStr);
        free(reversed8);
        return 1;
    }
    printf("Reversed value (8-bit) by bit by bit swapping technique in binary: %s\n", RevBinStr);
    printf("Reversed value (8-bit) in hex: 0x%X\n", *reversed8);

    // Free allocated memory
    free(binStr);
    free(RevBinStr);
    free(reversed8);

    return 0;
}
