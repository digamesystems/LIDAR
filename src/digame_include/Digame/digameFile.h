#ifndef __DIGAME_FILE_H__
#define __DIGAME_FILE_H__

#include <SPIFFS.h> 


//****************************************************************************************
// Grab contents from a file
//****************************************************************************************
String readFile(fs::FS &fs, const char * path){
    String retValue = "";
    //Serial.printf("Reading file: %s\r\n", path);

    File file = fs.open(path);
    if(!file || file.isDirectory()){
        DEBUG_PRINTLN("- failed to open file for reading");
        return retValue;
    }

    //DEBUG_PRINTLN("- read from file:");

    //retValue = file.readStringUntil('\r');
    
    while(file.available()){
        retValue = retValue + file.readString();
    }
    file.close();
    
    //DEBUG_PRINTLN(retValue);
    
    return retValue;
    
}

//****************************************************************************************
// Write text to a file
//****************************************************************************************
void writeFile(fs::FS &fs, const char * path, const char * message){
    //Serial.printf("Writing file: %s\r\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        DEBUG_PRINTLN("- failed to open file for writing");
        return;
    }
    if(file.print(message)){
        DEBUG_PRINTLN("  Saved.");
    } else {
        DEBUG_PRINTLN("- write failed");
    }
    file.close();
}

//****************************************************************************************
// Save some text to a file
//****************************************************************************************
void appendFile(fs::FS &fs, const char *filename, String contents)
{

  DEBUG_PRINTLN("Appending File...");

  // Open file for writing
  //debugUART.println("    Opening file for write...");
  File file = fs.open(filename, FILE_APPEND);

  if (!file)
  {
    DEBUG_PRINTLN(F("    Failed to open file!"));
    return;
  }

  //debugUART.println("    Writing file...");
  file.println(contents);

  // Close the file 
  DEBUG_PRINTLN("  Done.");
  file.close();
}



#endif //__DIGAME_FILE_H__