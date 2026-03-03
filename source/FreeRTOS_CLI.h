#ifndef FREERTOS_CLI_H
#define FREERTOS_CLI_H

#include <stddef.h>

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef BaseType_t (*pdCOMMAND_LINE_CALLBACK)(char *pcWriteBuffer,
                                               size_t xWriteBufferLen,
                                               const char *pcCommandString);

typedef struct xCOMMAND_LINE_INPUT
{
    const char *pcCommand;
    const char *pcHelpString;
    const pdCOMMAND_LINE_CALLBACK pxCommandInterpreter;
    int8_t cExpectedNumberOfParameters;
} CLI_Command_Definition_t;

BaseType_t FreeRTOS_CLIRegisterCommand(const CLI_Command_Definition_t *const pxCommandToRegister);
BaseType_t FreeRTOS_CLIProcessCommand(const char *const pcCommandInput, char *pcWriteBuffer, size_t xWriteBufferLen);
const char *FreeRTOS_CLIGetParameter(const char *pcCommandString,
                                     UBaseType_t uxWantedParameter,
                                     BaseType_t *pxParameterStringLength);

#ifdef __cplusplus
}
#endif

#endif /* FREERTOS_CLI_H */
