/***
 *** CSample.C
 ***
 *** An example program showing the use of LiveImport from purely straight C
 ***
 *** ----------
 ***
 *** Copyright 2009, Frontline Test Equipment, Inc
 ***
 *** ----------
 ***
 *** Tom Allebrandi (tallebrandi@fte.com/tom@ytram.com)
 *** Frontline Test Equipment, Inc
 ***/

/*******************************************************************************
 **
 ** Includes
 **/
#undef __STDC_WANT_SECURE_LIB__
#pragma warning(disable : 4115) // Avoid a warning from RpcAsync.h
#include    <windows.h>
#pragma warning(disable : 4115)
#include    <stdio.h>
#include <direct.h>
#include "LiveImportAPI.h"  // Definitions of function pointers.

/*******************************************************************************
 **
 ** Forward Declarations
 **/
bool checkLiveImportConnection(void);

/*******************************************************************************
 **
 ** Local Data
 **/
/*
 * This example program sends one Bluetooth HCI frame, an HCI_READ_BD_ADDR
 */
BYTE sampleFrame[] = { 0x09, 0x010, 0x00 };
bool g_boolSendCannedMessage = false;

/*******************************************************************************
 **
 ** Program
 **/

int liveimport_init()
{
    int             i;
    char            *pszConfiguration;
    int             status = ERROR_INVALID_FUNCTION;
    char            szConfiguration[1024];
    char            szConnectionString[1024];
    char            szIniFile[_MAX_PATH];
    bool boolQuit = false;
    int iFrameSize = sizeof(sampleFrame);
    char *psz = NULL;
    bool boolSuccess = false;
    HRESULT hr = S_OK;
    char szError[1024];
    char cPath[_MAX_PATH + 1];

    printf("Initializing...\n");

    /*
     * Set the name of the .INI file that is shared between FTS and a Live Import
     * data source
     */
    _tcscpy_s(szIniFile, _countof(szIniFile), "liveimport\\LiveImport.Ini");

    // Get the connection string from the shared .INI file
    GetPrivateProfileString("General", "ConnectionString", "", szConnectionString, sizeof(szConnectionString), szIniFile);

    if (_tcslen(szConnectionString) == 0)
    {
        fprintf(stderr, "[General]ConnectionString not found in %s.\n", szIniFile);
        return (status);
    }

    // Get the configuration data from the shared .INI file
    GetPrivateProfileSection("Configuration", szConfiguration, sizeof(szConfiguration), szIniFile);

    if (strlen(szConfiguration) == 0)
    {
        fprintf(stderr, "[Configuration] section not found in %s.\n", szIniFile);
        return (status);
    }

    /*
     * The return was a buffer of null terminated strings. Live Import wants a
     * single string in which each line terminated by a new line
     */
    pszConfiguration = szConfiguration;
    while ((i = strlen(pszConfiguration)) != 0)
    {
        pszConfiguration[i] = '\n'; // Change null character to new line
        pszConfiguration += (i + 1);    // Move to the next string
    }

    // Initialize the FTS LiveImport Server
    GetModuleFileName(GetModuleHandle(NULL), cPath, _MAX_PATH);

    _tcscpy_s(cPath, _countof(cPath), "liveimport\\LiveImportAPI.dll");
    g_pszLibraryName = cPath;

    if (!LoadLiveImportAPIFunctions())
        return (ERROR_INVALID_FUNCTION);

    hr = g_pInitializeLiveImport(szConnectionString, szConfiguration, &boolSuccess);

    if (FAILED(hr))             // Status flag from the server itself
    {
        _stprintf_s(szError, _countof(szError), "LiveImport.Initialize() failed with status 0x%x.\n", hr);
        MessageBox(NULL, szError, "Error", MB_OK);
        return (status);
    }
    else if (!boolSuccess)
    {
        _tcscpy_s(szError, _countof(szError), "LiveImport.Initialize() failed to initialize.\n");
        MessageBox(NULL, szError, "Error", MB_OK);
        return (status);
    }

    // Send one frame of data
    if (checkLiveImportConnection())
    {
        status = ERROR_SUCCESS;
    }
    return (status);
}

int liveimport_loghci(int dir, int size, char *data)
{
    SYSTEMTIME      systemtime;
    __int64         timestamp;
    int             status = ERROR_INVALID_FUNCTION;

    // Generate a timestamp
    GetLocalTime(&systemtime);
    SystemTimeToFileTime(&systemtime, (FILETIME *)&timestamp);
    // Send the frame
    if (FAILED(status = g_pSendFrame(
                            size - 1,
                            size - 1,
                            data + 1,
                            1 << ((*(data)) - 1),           // Bluetooth HCI Command
                            dir,                // Bluetooth Host
                            //  See BT Virtual.Dec and the [Configuration] in
                            //  the LiveImport.Ini file
                            timestamp)))
    {
        fprintf(stderr, "LiveImport_SendFrame() failed to send data: Error %d.", status);
        MessageBox(NULL, "LiveImport_SendFrame failed!", "Error", MB_OK);
        return (status);
    }
    return (status);
}

int liveimport_close()
{
    if (NULL != g_pReleaseLiveImport)
    {
        g_pReleaseLiveImport();
        NullLiveImportFunctionPointers();
    }

    FreeLibrary(g_hLiveImportAPI);
    return 0;
}
bool checkLiveImportConnection(void)
{
    bool boolConnectionIsRunning = false;
    char *pszMessage = NULL;
    int iCount = 0;
    DWORD dwStatus = 0;

    // Walk through the variables and return the state
    //  __asm int 3;
    while (!boolConnectionIsRunning && (iCount < 3))    // Wait no more than 3 seconds
    {
        dwStatus = g_pIsAppReady(&boolConnectionIsRunning);
        if (FAILED(dwStatus))
        {
            pszMessage = "Unspecified internal error in Live Import interface.";
            break;
        }

        if (!boolConnectionIsRunning)
        {
            Sleep(1000);
            iCount++;
        }
    }

    if ((100 == iCount) && !boolConnectionIsRunning)
        pszMessage = "FTS is not ready to receive data via Live Import.";

    if (boolConnectionIsRunning && g_boolSendCannedMessage)
    {
        pszMessage = "FTS is ready to receive data via Live Import.  Start capture now.";
        MessageBox(NULL, pszMessage, "Success", MB_OK);
        pszMessage = NULL;
    }

    // Display a message if there is a problem.
    if (pszMessage != NULL)
    {
        fprintf(stderr, "%s\n", pszMessage);
        MessageBox(NULL, pszMessage, "Error", MB_OK);
    }

    return ((bool)((NULL == pszMessage) ? true : false));   // Return true for "all is well"
}
