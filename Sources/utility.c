//std library includes
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include <string.h>

#include <utility.h>

bool strcmp_bool(const char* str1, const char* str2)
{
    return (0 == strcmp(str1, str2));
}

char* strtok_r(char* str, const char* delimStr, char** work)
{
    char* token;

    if (str != NULL) { //first token
        char* delimChar = strpbrk(str, delimStr);

        if (NULL == delimChar) {
            *work = NULL;

            token = str;
        }
        else {
            *delimChar = '\0';

            *work = (delimChar + 1);

            token = str;
        }
    }
    else { //next token
        if (NULL == (*work) || '\0' == (*work)) { //if the end of the string has been reached
            *work = NULL;

            token = NULL;
        }
        else {
            char* delimChar = strpbrk((*work), delimStr);

            if (NULL == delimChar) {
                token = (*work);

                *work = NULL;
            }
            else {
                *delimChar = '\0';

                token = (*work);

                *work = (delimChar + 1);
            }
        }
    }

    return token;
}

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

            head[i] = '\0';

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