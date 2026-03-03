#include "FreeRTOS_CLI.h"

#include <stdio.h>
#include <string.h>

#define CLI_MAX_COMMANDS       16U
#define CLI_INPUT_BUFFER_SIZE  256U

static const CLI_Command_Definition_t *s_cliCommands[CLI_MAX_COMMANDS];
static UBaseType_t s_cliCommandCount = 0U;
static const CLI_Command_Definition_t *s_cliActiveCommand = NULL;
static char s_cliInputCopy[CLI_INPUT_BUFFER_SIZE];

static size_t CliSkipSpaces(const char *text, size_t index, size_t len)
{
    while ((index < len) && ((text[index] == ' ') || (text[index] == '\t')))
    {
        index++;
    }
    return index;
}

static size_t CliTokenLength(const char *text, size_t index, size_t len)
{
    size_t start = index;

    while ((index < len) && (text[index] != ' ') && (text[index] != '\t') && (text[index] != '\r') &&
           (text[index] != '\n'))
    {
        index++;
    }
    return index - start;
}

static UBaseType_t CliCountParameters(const char *commandString)
{
    size_t len = strlen(commandString);
    size_t index = CliSkipSpaces(commandString, 0U, len);
    UBaseType_t count = 0U;

    if (index >= len)
    {
        return 0U;
    }

    index += CliTokenLength(commandString, index, len);
    while (index < len)
    {
        index = CliSkipSpaces(commandString, index, len);
        if (index >= len)
        {
            break;
        }
        index += CliTokenLength(commandString, index, len);
        count++;
    }

    return count;
}

BaseType_t FreeRTOS_CLIRegisterCommand(const CLI_Command_Definition_t *const pxCommandToRegister)
{
    UBaseType_t i;

    if ((pxCommandToRegister == NULL) || (pxCommandToRegister->pcCommand == NULL) ||
        (pxCommandToRegister->pxCommandInterpreter == NULL))
    {
        return pdFALSE;
    }

    for (i = 0U; i < s_cliCommandCount; i++)
    {
        if (strcmp(s_cliCommands[i]->pcCommand, pxCommandToRegister->pcCommand) == 0)
        {
            return pdFALSE;
        }
    }

    if (s_cliCommandCount >= CLI_MAX_COMMANDS)
    {
        return pdFALSE;
    }

    s_cliCommands[s_cliCommandCount] = pxCommandToRegister;
    s_cliCommandCount++;
    return pdTRUE;
}

const char *FreeRTOS_CLIGetParameter(const char *pcCommandString,
                                     UBaseType_t uxWantedParameter,
                                     BaseType_t *pxParameterStringLength)
{
    size_t len;
    size_t index;
    UBaseType_t current = 0U;

    if ((pcCommandString == NULL) || (uxWantedParameter == 0U))
    {
        return NULL;
    }

    len = strlen(pcCommandString);
    index = CliSkipSpaces(pcCommandString, 0U, len);
    if (index >= len)
    {
        return NULL;
    }

    index += CliTokenLength(pcCommandString, index, len);

    while (index < len)
    {
        size_t tokenStart;
        size_t tokenLen;

        index = CliSkipSpaces(pcCommandString, index, len);
        if (index >= len)
        {
            break;
        }

        tokenStart = index;
        tokenLen = CliTokenLength(pcCommandString, index, len);
        index += tokenLen;
        current++;

        if (current == uxWantedParameter)
        {
            if (pxParameterStringLength != NULL)
            {
                *pxParameterStringLength = (BaseType_t)tokenLen;
            }
            return &pcCommandString[tokenStart];
        }
    }

    return NULL;
}

BaseType_t FreeRTOS_CLIProcessCommand(const char *const pcCommandInput, char *pcWriteBuffer, size_t xWriteBufferLen)
{
    BaseType_t moreData = pdFALSE;

    if ((pcWriteBuffer == NULL) || (xWriteBufferLen == 0U))
    {
        return pdFALSE;
    }

    pcWriteBuffer[0] = '\0';

    if (s_cliActiveCommand == NULL)
    {
        const CLI_Command_Definition_t *found = NULL;
        size_t inputLen;
        size_t start;
        size_t commandLen;
        UBaseType_t i;

        if (pcCommandInput == NULL)
        {
            return pdFALSE;
        }

        inputLen = strlen(pcCommandInput);
        if (inputLen >= sizeof(s_cliInputCopy))
        {
            inputLen = sizeof(s_cliInputCopy) - 1U;
        }
        (void)memcpy(s_cliInputCopy, pcCommandInput, inputLen);
        s_cliInputCopy[inputLen] = '\0';

        start = CliSkipSpaces(s_cliInputCopy, 0U, inputLen);
        if (start >= inputLen)
        {
            return pdFALSE;
        }

        commandLen = CliTokenLength(s_cliInputCopy, start, inputLen);
        for (i = 0U; i < s_cliCommandCount; i++)
        {
            const char *name = s_cliCommands[i]->pcCommand;
            size_t nameLen = strlen(name);

            if ((nameLen == commandLen) && (strncmp(&s_cliInputCopy[start], name, nameLen) == 0))
            {
                found = s_cliCommands[i];
                break;
            }
        }

        if (found == NULL)
        {
            (void)snprintf(pcWriteBuffer, xWriteBufferLen, "Comando sconosciuto\r\n");
            return pdFALSE;
        }

        if (found->cExpectedNumberOfParameters >= 0)
        {
            UBaseType_t actualParams = CliCountParameters(s_cliInputCopy);
            if (actualParams != (UBaseType_t)found->cExpectedNumberOfParameters)
            {
                (void)snprintf(pcWriteBuffer, xWriteBufferLen, "%s", found->pcHelpString);
                return pdFALSE;
            }
        }

        s_cliActiveCommand = found;
    }

    moreData = s_cliActiveCommand->pxCommandInterpreter(pcWriteBuffer, xWriteBufferLen, s_cliInputCopy);
    if (moreData == pdFALSE)
    {
        s_cliActiveCommand = NULL;
    }
    return moreData;
}
