#pragma once

#include <iostream>
#include <string>
#include <streambuf>
#include <syslog.h>
#include <cstring>
#include <ostream>
#include <sstream>
#include <ctime>
#include <iomanip>

enum LogPriority{
    kLogEmerg   = LOG_EMERG,   // system is unusable
    kLogAlert   = LOG_ALERT,   // action must be taken immediately
    kLogCrit    = LOG_CRIT,    // critical conditions
    kLogErr     = LOG_ERR,     // error conditions
    kLogWarning = LOG_WARNING, // warning conditions
    kLogNotice  = LOG_NOTICE,  // normal, but significant, condition
    kLogInfo    = LOG_INFO,    // informational message
    kLogDebug   = LOG_DEBUG    // debug-level message
};

std::ostream& operator<< (std::ostream& os, const LogPriority& log_priority);

class Log : public std::basic_streambuf<char, std::char_traits<char> > {

public:
	
    explicit Log(std::string ident, int facility, bool toCOUT = false)
	{
		toCOUT_   = toCOUT;
	    facility_ = facility;
	    priority_ = LOG_DEBUG;
	    strncpy(ident_, ident.c_str(), sizeof(ident_));
	    ident_[sizeof(ident_)-1] = '\0';

	    openlog(ident_, LOG_PID, facility_);
	}
	
	void toCOUT(bool c)
	{
		toCOUT_ = c;
	}

protected:
    int sync()
	{
	    if (buffer_.length()) {
	        syslog(priority_, "%s", buffer_.c_str());
			
			if(toCOUT_) {
				std::time_t result = std::time(nullptr);
				std::cout << std::put_time(std::localtime(&result), "%b %d %T") << " "; // local time
				std::cout << buffer_.c_str();
			}
			    
			buffer_.erase();
	        priority_ = LOG_DEBUG; // default to debug for each message
	    }
	    return 0;
	}
	
    int overflow(int c)
	{
	    if (c != EOF) {
	        buffer_ += static_cast<char>(c);
	    } else {
	        sync();
	    }
	    return c;
	}

private:
    friend std::ostream& operator<< (std::ostream& os, const LogPriority& log_priority)
	{
	    static_cast<Log *>(os.rdbuf())->priority_ = (int)log_priority;
		
	    return os;
	}
	
    std::string buffer_;
    int facility_;
    int priority_;
	bool toCOUT_;
    char ident_[50];
};



class StreamLog : public std::ostringstream
{
   public:
	 StreamLog (std::string ident, int facility, bool toCOUT = false)
     {
          log = new Log (ident, facility, toCOUT);
          (static_cast<std::ostream*>(this))->rdbuf (log);
     }
	 
 	void toCOUT(bool c)
	{
 		log->toCOUT(c);
 	}
	
   private:
     Log* log;
};


namespace global {
	extern StreamLog* serverLog;
};
