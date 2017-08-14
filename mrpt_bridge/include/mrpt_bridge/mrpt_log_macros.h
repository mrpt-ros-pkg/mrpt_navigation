#ifndef MRPT_BRIDGE_LOG_MACROS_H
#define MRPT_BRIDGE_LOG_MACROS_H

#include <cstdarg>

#define MRPT_VIRTUAL_LOG_MACROS                     \
   protected:                                       \
	virtual void log_debug(const char* format, ...) \
	{                                               \
		va_list argptr;                             \
		va_start(argptr, format);                   \
		fprintf(stdout, "DEBUG: ");                 \
		vfprintf(stdout, format, argptr);           \
		fprintf(stdout, "\n");                      \
		va_end(argptr);                             \
	}                                               \
                                                    \
   protected:                                       \
	virtual void log_info(const char* format, ...)  \
	{                                               \
		va_list argptr;                             \
		va_start(argptr, format);                   \
		fprintf(stdout, "INFO: ");                  \
		vfprintf(stdout, format, argptr);           \
		fprintf(stdout, "\n");                      \
		va_end(argptr);                             \
	}                                               \
                                                    \
   protected:                                       \
	virtual void log_warn(const char* format, ...)  \
	{                                               \
		va_list argptr;                             \
		va_start(argptr, format);                   \
		fprintf(stderr, "WARNING: ");               \
		vfprintf(stderr, format, argptr);           \
		fprintf(stderr, "\n");                      \
		va_end(argptr);                             \
	}                                               \
                                                    \
   protected:                                       \
	virtual void log_error(const char* format, ...) \
	{                                               \
		va_list argptr;                             \
		va_start(argptr, format);                   \
		fprintf(stderr, "ERROR: ");                 \
		vfprintf(stderr, format, argptr);           \
		fprintf(stderr, "\n");                      \
		va_end(argptr);                             \
	}

#define MRPT_ROS_LOG_MACROS                         \
   protected:                                       \
	virtual void log_debug(const char* format, ...) \
	{                                               \
		char buffer[0x1FF];                         \
		va_list argptr;                             \
		va_start(argptr, format);                   \
		vsprintf(buffer, format, argptr);           \
		ROS_DEBUG("%s", buffer);                    \
		va_end(argptr);                             \
	}                                               \
                                                    \
   protected:                                       \
	virtual void log_info(const char* format, ...)  \
	{                                               \
		char buffer[0x1FF];                         \
		va_list argptr;                             \
		va_start(argptr, format);                   \
		vsprintf(buffer, format, argptr);           \
		ROS_INFO("%s", buffer);                     \
		va_end(argptr);                             \
	}                                               \
                                                    \
   protected:                                       \
	virtual void log_warn(const char* format, ...)  \
	{                                               \
		char buffer[0x1FF];                         \
		va_list argptr;                             \
		va_start(argptr, format);                   \
		vsprintf(buffer, format, argptr);           \
		ROS_WARN("%s", buffer);                     \
		va_end(argptr);                             \
	}                                               \
                                                    \
   protected:                                       \
	virtual void log_error(const char* format, ...) \
	{                                               \
		char buffer[0x1FF];                         \
		va_list argptr;                             \
		va_start(argptr, format);                   \
		vsprintf(buffer, format, argptr);           \
		ROS_ERROR("%s", buffer);                    \
		va_end(argptr);                             \
	}

#endif  // MRPT_BRIDGE_LOG_MACROS_H
