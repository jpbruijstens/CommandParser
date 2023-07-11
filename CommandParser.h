#ifndef __COMMANDPARSER_H_
#define __COMMANDPARSER_H_

#include <Arduino.h>

class CommandParser {
  public:
    typedef struct cmd_t {
      const char *cmd;
      void (* const cb) (char*arg1,char*arg2, char*arg3);
    } cmd_t;
    
    CommandParser( const cmd_t *cmdlist );
    void process( Stream *str);
    void listCommands( Stream *str);
    
  private:
    const cmd_t *m_cmdlist;
    uint8_t m_len;
    char m_line[30];
    
    void processLine(Stream *str);
};

#endif
