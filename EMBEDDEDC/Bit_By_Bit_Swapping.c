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

#define BYTE_SIZE 1
#define HALF_WORD_SIZE 2
#define WORD_SIZE 4
#define MAX_BITS 32 // Maximum number of bits supported
#define SIZE_OF(var) ((char*)(&var + 1) - (char*)(&var))

// Function to convert a decimal number to binary string (static allocation)
void DecToBin(void* num, size_t size, char* bin) {
    int bit_count = size * 8; // Calculate the number of bits
    uint32_t n = 0;

    switch (size) {
        case BYTE_SIZE:
            n = *((uint8_t*)num);
            break;
        case HALF_WORD_SIZE:
            n = *((uint16_t*)num);
            break;
        case WORD_SIZE:
            n = *((uint32_t*)num);
            break;
        default:
            fprintf(stderr, "Unsupported size in DecToBin function\n");
            return;
    }

    for (int idx = bit_count - 1; idx >= 0; idx--) {
        bin[bit_count - 1 - idx] = (n & (1 << idx)) ? '1' : '0'; // Set the corresponding index in the string
    }
    bin[bit_count] = '\0'; // Set the null terminator at the end of the string
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

// Function to reverse bits of a number (dynamic allocation for the result)
void* reverseBits(void* num, size_t size) {
    uint32_t n = 0;

    switch (size) {
        case BYTE_SIZE:
            n = *((uint8_t*)num);
            break;
        case HALF_WORD_SIZE:
            n = *((uint16_t*)num);
            break;
        case WORD_SIZE:
            n = *((uint32_t*)num);
            break;
        default:
            fprintf(stderr, "Unsupported size in reverseBits function\n");
            return NULL;
    }

    uint32_t reversed = reverseBitsHelper(n, size * 8);
    void* result = NULL;

    switch (size) {
        case BYTE_SIZE:
            result = malloc(sizeof(uint8_t));
            if (result != NULL) {
                *((uint8_t*)result) = (uint8_t)reversed;
            }
            break;
        case HALF_WORD_SIZE:
            result = malloc(sizeof(uint16_t));
            if (result != NULL) {
                *((uint16_t*)result) = (uint16_t)reversed;
            }
            break;
        case WORD_SIZE:
            result = malloc(sizeof(uint32_t));
            if (result != NULL) {
                *((uint32_t*)result) = (uint32_t)reversed;
            }
            break;
        default:
            fprintf(stderr, "Unsupported size in reverseBits function\n");
            return NULL;
    }

    return result;
}

int main() {
    // Static allocation for binary string representation
    char binStr[MAX_BITS + 1]; // +1 for the null terminator

    // Example with 32-bit number
    uint32_t x = 0xDEADBEEF;
    DecToBin(&x, SIZE_OF(x), binStr);
    printf("Original value in binary: %s\n", binStr);
    printf("Original value in hex: 0x%X\n", x);

    uint32_t* reversed32 = (uint32_t*)reverseBits(&x, SIZE_OF(x));
    if (reversed32 == NULL) {
        fprintf(stderr, "Failed to reverse bits in main function\n");
        return 1;
    }
    DecToBin(reversed32, SIZE_OF(*reversed32), binStr);
    printf("Reversed value by bit by bit swapping technique in binary: %s\n", binStr);
    printf("Reversed value in hex: 0x%X\n", *reversed32);

    // Free allocated memory
    free(reversed32);
    reversed32 = NULL;

    // Example with 16-bit number
    uint16_t y = 0xABCD;
    DecToBin(&y, SIZE_OF(y), binStr);
    printf("Original value (16-bit) in binary: %s\n", binStr);
    printf("Original value (16-bit) in hex: 0x%X\n", y);

    uint16_t* reversed16 = (uint16_t*)reverseBits(&y, SIZE_OF(y));
    if (reversed16 == NULL) {
        fprintf(stderr, "Failed to reverse bits in main function\n");
        return 1;
    }
    DecToBin(reversed16, SIZE_OF(*reversed16), binStr);
    printf("Reversed value (16-bit) by bit by bit swapping technique in binary: %s\n", binStr);
    printf("Reversed value (16-bit) in hex: 0x%X\n", *reversed16);

    // Free allocated memory
    free(reversed16);
    reversed16 = NULL;

    // Example with 8-bit number
    uint8_t z = 0xAB;
    DecToBin(&z, SIZE_OF(z), binStr);
    printf("Original value (8-bit) in binary: %s\n", binStr);
    printf("Original value (8-bit) in hex: 0x%X\n", z);

    uint8_t* reversed8 = (uint8_t*)reverseBits(&z, SIZE_OF(z));
    if (reversed8 == NULL) {
        fprintf(stderr, "Failed to reverse bits in main function\n");
        return 1;
    }
    DecToBin(reversed8, SIZE_OF(*reversed8), binStr);
    printf("Reversed value (8-bit) by bit by bit swapping technique in binary: %s\n", binStr);
    printf("Reversed value (8-bit) in hex: 0x%X\n", *reversed8);

    // Free allocated memory
    free(reversed8);
    reversed8 = NULL;

    return 0;
}
