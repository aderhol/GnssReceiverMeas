//std library includes
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

//user includes
#include <nmea.h>




bool addChecksum(char str[], size_t maxLength, bool hasStartChar, bool hasEndChar, bool addNewLine)
{
    bool OK;

    if(maxLength >= 7){ //if str isn't too short to fit the most basic NMEA message into it
        size_t currLength = strlen(str); //get the current length of 'str'

        size_t requiredSize = currLength +
                (hasStartChar ? 0 : 1) +
                (hasEndChar ? 0 : 1) +
                (addNewLine ? 2 : 0) +
                2; //the checksum takes up 2 chars

        if(maxLength >= requiredSize){ //if it will fit
            if(!hasStartChar){ //if it doesn't have the starting $
                //shift the string by 1 to the right to free up the first char
                char* ptr = str;

                char this = '$', prv;
                do{
                    prv = this; //last char is the previous char
                    this = *ptr; //save the char
                    *(ptr++) = prv; //overwrite this, go to next char
                }while(prv != '\0');
            }

            str++; //ignore the staring *

            if(!hasEndChar){ //if it doesn't have the closing *
                strncat(str, "*", 1); //append the closing *
            }

            uint8_t checksum = 0;
            while(str[0] != '*'){ //go until the end of the sequence
                checksum ^= ((uint8_t)str[0]);

                str++; //ignore the char already XORed
            }

            //checksum to HEX string
            char checksumStr[3];
            checksumStr[0] = checksum / 16;
            checksumStr[1] = checksum % 16;
            checksumStr[2] = '\0';

            int_fast8_t i;
            for(i = 0; i < 2; i++){
                if(((uint8_t)checksumStr[i]) < 10){
                    checksumStr[i] = '0' + checksumStr[i];
                }
                else{
                    checksumStr[i] = 'A' + (checksumStr[i] - 10);
                }
            }

            strncat(str, checksumStr, 2); //append the checksum

            if(addNewLine){
                strncat(str, "\r\n", 2); //append the newLine
            }

            OK = true;
        }
        else{ //if it won't fit
            OK = false;
        }
    }
    else{ //if str is too short for anything at all
        OK = false;
    }

    return OK;
}

