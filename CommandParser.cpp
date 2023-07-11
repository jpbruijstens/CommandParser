#include "CommandParser.h"
#include <string.h>

CommandParser::CommandParser (const cmd_t *cmdlist ) : m_cmdlist ( cmdlist ), m_len ( 0 ) {
  m_line[0] = 0;
}

void CommandParser::process(Stream *str) {
  while ( str->available() > 0 ) {
    char b = str->read();
    switch ( b ) {
        case '\r':
        case '\n':
          if ( m_len ) {
            processLine(str);
            m_len = 0;
          }
          break;
          
        default:
          if ( m_len < ( sizeof(m_line) - 1 ) ) {
            m_line[m_len] = b;
            m_line[m_len + 1] = 0;
            ++m_len;
          } else {
            str->print ( "ERROR input too large: " );
            str->println ( (int) m_len );
            m_len = 0;
            m_line[0] = 0;
          }
          break;
    }
  }
}

void CommandParser::processLine(Stream *str) {
  char *cmd = strtok ( m_line, " " );
  char *arg1 = strtok ( NULL, " " );
  char *arg2 = strtok ( NULL, " " );
  char *arg3 = strtok ( NULL, " " );
  
  const cmd_t *cur = m_cmdlist;
  
  while ( cur->cb ) {
    if ( strcasecmp ( cur->cmd, cmd ) == 0 ) {
      cur->cb ( arg1, arg2, arg3 );
      return;
    }
    ++cur;
  }
  str->print ( "ERROR cmd '" );
  str->print ( cmd );
  str->println ( "' not supported" );
}

void CommandParser::listCommands(Stream *str) {
  str->println ( "Supported commands:" );
  const cmd_t *cur = m_cmdlist;
  while ( cur->cb ) {
    str->println ( cur->cmd );
    ++cur;
  }
}
