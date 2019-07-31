//std library includes
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include <string.h>

#include <utility.h>

bool appendFloat(char* str, float val, size_t precision, size_t maxLength, bool addComma)
{
    if((1 + 1 + precision) > maxLength){ //X.X...X
        return false;
    }

    if(isnan(val)){ //if NaN
        if(3 <= maxLength){
            strcat(str, "NaN");

            return true;
        }
        else{
            return false;
        }
    }

    if(isinf(val)){ //if inf
        if(3 <= maxLength){
            strcat(str, "inf");

            return true;
        }
        else{
            return false;
        }
    }

    size_t i;

    for(; (*str) != '\0'; str++); //get to the end
    char* head = str;


    for(i = 0; i < precision; i++){
        val *= 10.0f;
    }

    //rounding
    if(val >= 0){
        val += 0.5f;
    }
    else{
        val -= 0.5f;
    }


    if(fabsf(val) > ((float)INT64_MAX)){ //if the float is too big
        return false;
    }
    else{
        int64_t iVal = (int64_t)val;

        i = 0;

        if(iVal < 0){ //if negative
            *(head++) = '-';
            maxLength--;
            iVal *= -1;
        }

        if(0 == iVal){
            head[i++] ='0';
            if(precision > 0){
                head[i++] ='.';
            }

            size_t j;
            for(j = 0; j < precision; j++){
                head[i++] = '0';
            }

            return true;
        }

        while(0 != iVal && i < maxLength){
            head[i++] = '0' + (iVal % 10);
            iVal /= 10;

            if(precision == i){
                head[i++] = '.';
                head[i++] = '0' + (iVal % 10);
                iVal /= 10;
            }
        }

        head[i] = '\0';

        if(i >= maxLength && 0 != iVal){ //if the number was too long
            return false;
        }else{ //reverse the order
            char* A = head;
            char* B = head + (i - 1);

            for(; A < B;(A++, B--)){
                char ch = *A;
                *A = *B;
                *B = ch;
            }

            if(addComma){
                head[i++] = ',';
                head[i] = '\0';
            }

            return true;
        }
    }
}
