//Reverse words in a string
// i/p : I am Awesome
//o/p : Awesome am I
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
int get_string_length(const char* str) {
    int count = 0;
    while (str[count] != 0) {
        count++;
    }   
    return count;
}
void reverse_word(char str[],int len)
{
    char temp=0;
    int i=0;
    while(i<len)
    {
        temp=str[i];
        str[i]=str[len];
        str[len]=temp;
        i++;
        len--;
    }
}

void reverse_words(char str[],int j,int k)
{
    char temp=0;

    while(j<k)
    {
        temp=str[j];
        str[j]=str[k];
        str[k]=temp;
        j++;
        k--;
    }
}
int main()
{  char str1[] = "Bangalore";
   char str[]="I am awesome";
  //main pointer
   int i=0;
   //temp first
   int j=0;
  //last pointer (length)
   int k =get_string_length(str)-1;
   int len =get_string_length(str1)-1;
   printf("Org str: %s\n",str);
   printf("Org str1: %s\n",str1);
    while(str[i]!='\0')
   {
       if (str[i]==' ')
       {
           k=i-1;
           reverse_words(str,j,k);
           j=i+1;//next word pointer 
    }

      ++i;
   }
   //Reverse last word
   reverse_words(str, j, i - 1);
   //Reverse whole sentence
   reverse_words(str, 0, get_string_length(str) - 1);
  
   reverse_word(str1,len);
   printf("Rev str1: %s\n",str1);
  printf("Reversed sentence: %s\n", str);

  
  return 0;
}