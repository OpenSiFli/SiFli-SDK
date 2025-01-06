/**
Usage:

1. Copy this file to OzoneDir/Plugins/OS/RtThreadOSPlugin.js
2. Restart Ozone, and attach to target
2. Run "Project.SetOSPlugin("RtThreadOSPlugin");" console cmd in Ozone
3. Click "View"->"RtThreadRTOS" on tool bar



*/

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/

/**** Configurable **************************************************/
var StackCheckLimit = 8192;

/**** ARM Register indices, constant ********************************/
var arm_r0  =  0;
var arm_r1  =  1;
var arm_r2  =  2;
var arm_r3  =  3;
var arm_r4  =  4;
var arm_r5  =  5;
var arm_r6  =  6;
var arm_r7  =  7;
var arm_r8  =  8;
var arm_r9  =  9;
var arm_r10 = 10;
var arm_r11 = 11;
var arm_r12 = 12;
var arm_sp  = 13;
var arm_lr  = 14;
var arm_pc  = 15;
var arm_psr = 16;

//
// RtThreadRTOS preserves an additional [pseudo] register
// (exec location with ARM and a modified LR with Cortex-M)
//
var arm_exec = 15;

/**** RISC-V Register indices, constant ********************************/
var riscv_pc  =  0;
var riscv_ra  =  1;
var riscv_sp  =  2;
var riscv_gp  =  3;
var riscv_tp  =  4;
var riscv_t0  =  5;
var riscv_t1  =  6;
var riscv_t2  =  7;
var riscv_s0  =  8;
var riscv_s1  =  9;
var riscv_a0  = 10;
var riscv_a1  = 11;
var riscv_a2  = 12;
var riscv_a3  = 13;
var riscv_a4  = 14;
var riscv_a5  = 15;
var riscv_a6  = 16;
var riscv_a7  = 17;
var riscv_s2  = 18;
var riscv_s3  = 19;
var riscv_s4  = 20;
var riscv_s5  = 21;
var riscv_s6  = 22;
var riscv_s7  = 23;
var riscv_s8  = 24;
var riscv_s9  = 25;
var riscv_s10 = 26;
var riscv_s11 = 27;
var riscv_t3  = 28;
var riscv_t4  = 29;
var riscv_t5  = 30;
var riscv_t6  = 31;

/**** embOS task states, constant ***********************************/
var RT_THREAD_INIT                =  0x00;  //               /**< Initialized status */
var RT_THREAD_READY               =  0x01;  //               /**< Ready status */
var RT_THREAD_SUSPEND             =  0x02;  //               /**< Suspend status */
var RT_THREAD_RUNNING             =  0x03;  //               /**< Running status */
var RT_THREAD_BLOCK               =  0x02;  //               /**< Blocked status */
var RT_THREAD_CLOSE               =  0x04;  //               /**< Closed status */
var RT_THREAD_STAT_MASK           =  0x0f;  //

var RT_THREAD_STAT_SIGNAL         =  0x10;  //               /**< task hold signals */
var RT_THREAD_STAT_SIGNAL_READY   =  0x11;  //
var RT_THREAD_STAT_SIGNAL_WAIT    =  0x20;  //               /**< task is waiting for signals */
var RT_THREAD_STAT_SIGNAL_PENDING =  0x40;  //               /**< signals is held and it has not been procressed */
var RT_THREAD_STAT_SIGNAL_MASK    =  0xf0;  //


/**** embOS list names, constant ************************************/
var Timers              = "Timers";
var Mailboxes           = "Mailboxes";
var Queues              = "Queues";
var Mutexes             = "Mutexes";
var Semaphores          = "Semaphores";
var MemoryPools         = "Memory Pools";
var EventObjects        = "Event Objects";
var WatchDogs           = "Watchdogs";
var SystemInformation   = "System Information";

/**** embOS object types  *******************************************/
var CastMutex           = "(OS_MUTEX*)";
var CastSema            = "(OS_SEMAPHORE*)";
var CastQueue           = "(OS_QUEUE*)";
var CastMEMF            = "(OS_MEMPOOL*)";
var RootMutex           = "OS_pMutexRoot";
var RootSema            = "OS_pSemaRoot";

/*** Global vars ****************************************************/
var OS_Global;

/*********************************************************************
*
*       Local functions
*
**********************************************************************
*/
/*********************************************************************
*
*       GetObjectName()
*
*  Parameters
*    Address: Object address
*
*  Functions description:
*    Returns an object name
*/
function GetObjectName(Address) {
  var p;
  var pObjID;

  p = Debug.evaluate("OS_pObjNameRoot");
  if (p == undefined || p == 0) {
    return "";
  }
  pObjID = Debug.evaluate("(*(OS_OBJNAME*)" + p + ").pOSObjID");
  while (pObjID != Address) {
    p = Debug.evaluate("(OS_OBJNAME*)(*(OS_OBJNAME*)" + p + ").pNext");
    if (p != undefined && p != 0) {
      pObjID = Debug.evaluate("(*(OS_OBJNAME*)" + p + ").pOSObjID");
    } else {
      return "";
    }
  }
  return Debug.evaluate("(char*)(*(OS_OBJNAME*)" + p + ").sName");
}

function GetTCB(pTaskNode)
{
    var offset = Debug.evaluate("((char *) &(((struct rt_thread *) " + pTaskNode + ")->list)) - ((char *)" + pTaskNode + ")"); 
    var OS_TASK = Debug.evaluate("*(struct rt_thread *) (((char *)" + pTaskNode + ") - " + offset + ")"); 
    
    return OS_TASK;
}

function GetpTCB(pTaskNode)
{
    var offset = Debug.evaluate("((char *) &(((struct rt_thread *) " + pTaskNode + ")->list)) - ((char *)" + pTaskNode + ")"); 
    var p_OS_TASK = Debug.evaluate("(struct rt_thread *) (((char *)" + pTaskNode + ") - " + offset + ")"); 
    
    return p_OS_TASK;
}
/*********************************************************************
*
*       GetTaskName()
*
*  Parameters
*    Address: Task address
*
*  Functions description:
*    Returns an object name
*/
function GetTaskName(Address) {
  var Name;
  
  var p_OS_TASK = GetpTCB(Address);

  Name = Debug.evaluate("(char*)(*(struct rt_thread *) " + p_OS_TASK + ").name");
  if (Name == undefined) {
    Name = "N/A";
  } else if (Name == 0) {
    Name = "";
  }
  return Name;
}

/*********************************************************************
*
*       GetWaitingTasks()
*
*  Functions description:
*    Used to retrieve a string with a list of tasks that are waiting
*    for the specified object.
*
*  Parameters
*    pObject: Address of the object which the tasks returned are waiting on.
*/
function GetWaitingTasks(pObject) {
  var WaitingTasks = "";
  var TaskIterator;
  var pWaitObject;

  for (TaskIterator = OS_Global.pTask; (TaskIterator != undefined) && (TaskIterator != 0); TaskIterator = Debug.evaluate("(OS_TASK*)(*(OS_TASK*)" + TaskIterator + ").pNext")) {
      
    pWaitObject = Debug.evaluate("(OS_WAIT_OBJ_STRUCT*)(*(OS_WAIT_LIST*)(*(OS_TASK*)" + TaskIterator + ").pWaitList).pWaitObj");
    
    if (pWaitObject == pObject) {
      WaitingTasks += "0x" + TaskIterator.toString(16).toUpperCase() + " (" + GetTaskName(TaskIterator) + "), ";
    }
  }
  WaitingTasks = WaitingTasks.substring(0, WaitingTasks.length - 2);
  return WaitingTasks;
}

/*********************************************************************
*
*       UpdateTimers()
*
*  Functions description:
*    Called from the update() function in order to retrieve timer
*    information and update the timers list.
*
*  Parameters
*    Window: Object used to access the plug in window.
*/
function UpdateTimers(Window) {
  var Hook;
  var RemainingTime;
  var Timeout;
  var Period;
  var pTimer;
  var Timer;
  var CallbackName;

  for (pTimer = OS_Global.pTimer; (pTimer != undefined) && (pTimer != 0); pTimer = Debug.evaluate("(*(OS_TIMER*)" + pTimer + ").pNext")) {
    //
    // Read information from timer structure. Only running timers can be listed.
    //
    Timer = Debug.evaluate("*(OS_TIMER_EX*)" + pTimer);
    if (Timer == undefined) {
      //
      // If OS_TIMER_EX can't be evaluated (because there's no debug information for it)
      //
      Timer = Debug.evaluate("*(OS_TIMER*)" + pTimer);
      Hook          = Timer.Hook;
      Timeout       = Timer.Time;
      RemainingTime = Timeout - OS_Global.Time;
      Period        = Timer.Period;
    } else {
      //
      // OS_TIMER_EX is used to retrieve information for extended and normal timers
      //
      Hook          = Timer.Timer.Hook;
      Timeout       = Timer.Timer.Time;
      RemainingTime = Timeout - OS_Global.Time;
      Period        = Timer.Timer.Period;
    }
    //
    // If the hook function is _OS_cbTimerEx, read the user callback
    // address from the extended timer structure and get its name.
    //
    CallbackName  = Debug.getfunction(Hook);
    if (CallbackName == "_OS_cbTimerEx") {
      //
      // Timer is expected to be an object of OS_TIMER_EX, as it has
      // the extended callback function as hook.
      //
      Hook         = Timer.pfUser;
      CallbackName = Debug.getfunction(Hook);
    }
    //
    // Update information in the plug in window
    //
    Window.add2(Timers,
                "0x" + pTimer.toString(16).toUpperCase(),
                GetObjectName(pTimer),
                "0x" + Hook.toString(16).toUpperCase() + " (" + CallbackName + ")",
                RemainingTime + " (" + Timeout + ")",
                Period);
  }
}

/*********************************************************************
*
*       UpdateMailboxes()
*
*  Functions description:
*    Called from the update() function in order to retrieve mailbox
*    information and update the mailbox list.
*
*  Parameters
*    Window: Object used to access the plug in window.
*/
function UpdateMailboxes(Window) {
  var InUse;
  var Messages;
  var MaxMessages;
  var MessageSize;
  var pBuffer;
  var WaitingTasks;
  var pMailbox;
  var MailBox;

  for (pMailbox = Debug.evaluate("OS_pMailboxRoot"); (pMailbox != undefined) && (pMailbox != 0); pMailbox = Debug.evaluate("(OS_MAILBOX*)(*(OS_MAILBOX*)" + pMailbox + ").pNext")) {
      
    MailBox = Debug.evaluate("*(OS_MAILBOX*)" + pMailbox);
    //
    // Read information from mailbox structure
    //
    InUse       = MailBox.InUse;
    Messages    = MailBox.nofMsg;
    MaxMessages = MailBox.maxMsg;
    MessageSize = MailBox.sizeofMsg;
    pBuffer     = Debug.evaluate("(unsigned int*)(*(OS_MAILBOX*)" + pMailbox + ").pData");
    //
    // Retrieve waiting tasks
    //
    WaitingTasks = GetWaitingTasks(pMailbox);
    //
    // Update information in the plug in window
    //
    Window.add2(Mailboxes,
                "0x" + pMailbox.toString(16).toUpperCase(),
                GetObjectName(pMailbox),
                Messages + "/" + MaxMessages,
                MessageSize,
                "0x" + pBuffer.toString(16).toUpperCase(),
                WaitingTasks,
                InUse ? "True" : "False");
  }
}

/*********************************************************************
*
*       UpdateQueues()
*
*  Functions description:
*    Called from the update() function in order to retrieve queue
*    information and update the queues list.
*
*  Parameters
*    Window: Object used to access the plug in window.
*/
function UpdateQueues(Window) {
  var Messages;
  var pBuffer;
  var BufferSize;
  var WaitingTasks;
  var pQueue;
  var Queue;

  for (pQueue = Debug.evaluate("OS_pQRoot"); (pQueue != undefined) && (pQueue != 0); pQueue = Debug.evaluate("(*" + CastQueue + pQueue + ").pNext")) {
      
    Queue = Debug.evaluate("*" + CastQueue + pQueue);
    //
    // Read information from queue structure
    //
    Messages   = Queue.MsgCnt;
    pBuffer    = Queue.pData;
    BufferSize = Queue.Size;
    //
    // Retrieve waiting tasks
    //
    WaitingTasks = GetWaitingTasks(pQueue);
    //
    // Update information in the plug in window
    //
    Window.add2(Queues,
                "0x" + pQueue.toString(16).toUpperCase(),
                GetObjectName(pQueue),
                Messages,
                "0x" + pBuffer.toString(16).toUpperCase(),
                BufferSize,
                WaitingTasks);
  }
}

/*********************************************************************
*
*       UpdateMutexes()
*
*  Functions description:
*    Called from the update() function in order to retrieve mutexes
*    information and update the resource semaphores list.
*
*  Parameters
*    Window: Object used to access the plug in window.
*/
function UpdateMutexes(Window) {
  var Owner;
  var UseCounter;
  var WaitingTasks;
  var pMutex;
  var Mutex;

  for (pMutex = Debug.evaluate(RootMutex); (pMutex != undefined) && (pMutex != 0); pMutex = Debug.evaluate("(*" + CastMutex + pMutex + ").pNext")) {
      
    Mutex = Debug.evaluate("*" + CastMutex + pMutex);
    //
    // Read information from queue structure
    //
    UseCounter = Mutex.UseCnt;
    if (UseCounter != 0) {
      Owner = Mutex.pTask;
    } else {
      Owner = 0;
    }
    //
    // Retrieve waiting tasks
    //
    WaitingTasks = GetWaitingTasks(pMutex);
    //
    // Update information in the plug in window
    //
    Window.add2(Mutexes,
                "0x" + pMutex.toString(16).toUpperCase(),
                GetObjectName(pMutex),
                Owner ? "0x" + Owner.toString(16).toUpperCase() + " (" + GetTaskName(Owner) + ")" : "",
                UseCounter,
                WaitingTasks);
  }
}

/*********************************************************************
*
*       UpdateSemaphores()
*
*  Functions description:
*    Called from the update() function in order to retrieve semaphores
*    information and update the semaphores list.
*
*  Parameters
*    Window: Object used to access the plug in window.
*/
function UpdateSemaphores(Window) {
  var Count;
  var WaitingTasks;
  var pSema;

  for (pSema = Debug.evaluate(RootSema); (pSema != undefined) && (pSema != 0); pSema = Debug.evaluate("(*" + CastSema + pSema + ").pNext")) {
    //
    // Read information from queue structure
    //
    Count = Debug.evaluate("(*" + CastSema + pSema + ").Cnt");
    //
    // Retrieve waiting tasks
    //
    WaitingTasks = GetWaitingTasks(pSema);
    //
    // Update information in the plug in window
    //
    Window.add2(Semaphores,
                "0x" + pSema.toString(16).toUpperCase(),
                GetObjectName(pSema),
                Count,
                WaitingTasks);
  }
}

/*********************************************************************
*
*       UpdateMemoryPools()
*
*  Functions description:
*    Called from the update() function in order to retrieve memory
*    pool information and update the memory pools list.
*
*  Parameters
*    Window: Object used to access the plug in window.
*/
function UpdateMemoryPools(Window) {
  var NumFreeBlocks;
  var NumBlocks;
  var BlockSize;
  var MaxUsage;
  var pBuffer;
  var WaitingTasks;
  var pMemPool;
  var MemPool;

  for (pMemPool = Debug.evaluate("OS_pMEMFRoot"); (pMemPool != undefined) && (pMemPool != 0); pMemPool = Debug.evaluate("(*" + CastMEMF + pMemPool + ").pNext")) {
      
    MemPool = Debug.evaluate("*" + CastMEMF + pMemPool);
    //
    // Read information from queue structure
    //
    NumFreeBlocks = MemPool.NumFreeBlocks;
    NumBlocks     = MemPool.NumBlocks;
    BlockSize     = MemPool.BlockSize;
    MaxUsage      = MemPool.MaxUsed;
    pBuffer       = MemPool.pPool;
    //
    // Retrieve waiting tasks
    //
    WaitingTasks = GetWaitingTasks(pMemPool);
    //
    // Update information in the plug in window
    //
    Window.add2(MemoryPools,
                "0x" + pMemPool.toString(16).toUpperCase(),
                GetObjectName(pMemPool),
                NumFreeBlocks + "/" + NumBlocks,
                BlockSize,
                MaxUsage,
                "0x" + pBuffer.toString(16).toUpperCase(),
                WaitingTasks);
  }
}

/*********************************************************************
*
*       UpdateEventObjects()
*
*  Functions description:
*    Called from the update() function in order to retrieve event
*    object information and update the event objects list.
*
*  Parameters
*    Window: Object used to access the plug in window.
*/
function UpdateEventObjects(Window) {
  var Signaled;
  var ResetMode;
  var MaskMode;
  var sMaskMode;
  var sResetMode;
  var WaitingTasks;

  for (var pEvent = Debug.evaluate("OS_pEventRoot"); (pEvent != undefined) && (pEvent != 0); pEvent = Debug.evaluate("(*(OS_EVENT*)" + pEvent + ").pNext")) {
    //
    // Read information from queue structure
    //
    Signaled  = Debug.evaluate("     (*(OS_EVENT*)" + pEvent + ").Signaled");
    ResetMode = Debug.evaluate("(int)(*(OS_EVENT*)" + pEvent + ").ResetMode");
    MaskMode  = Debug.evaluate("(int)(*(OS_EVENT*)" + pEvent + ").MaskMode");
    //
    // set sMaskMode
    //
    switch (ResetMode) {
      case 0:  sMaksMode = "OR Logic";
      case 4:  sMaskMode = "AND Logic";
      default: sMaskMode = "0x" + MaskMode.toString(16).toUpperCase();
    }
    //
    // set sResetMode
    //
    switch (ResetMode) {
      case 0:  sResetMode = "Semiauto";
      case 1:  sResetMode = "Manual";
      case 2:  sResetMode = "Auto";
      default: sResetMode = "0x" + ResetMode.toString(16).toUpperCase();
    }
    //
    // Retrieve waiting tasks
    //
    WaitingTasks = GetWaitingTasks(pEvent);
    //
    // Update information in the plug in window
    //
    Window.add2(EventObjects,
                "0x" + pEvent.toString(16).toUpperCase(),
                GetObjectName(pEvent),
                "0x" + Signaled.toString(16).toUpperCase(),
                sResetMode,
                sMaskMode,
                WaitingTasks);
  }
}

/*********************************************************************
*
*       UpdateWatchDogs()
*
*  Functions description:
*    Called from the update() function in order to retrieve watchdog
*    information and update the watchdogs list.
*
*  Parameters
*    Window: Object used to access the plug in window.
*/
function UpdateWatchDogs(Window) {
  var TimeDex;
  var Period;
  var GlobalTime;
  var Watchdog;
  var pWatchdog;

  GlobalTime = OS_Global.Time;
  
  for (pWatchdog = Debug.evaluate("OS_pWDRoot"); (pWatchdog != undefined) && (pWatchdog != 0); pWatchdog = Debug.evaluate("(*(OS_WD*)" + pWatchdog + ").pNext")) {
    
    Watchdog = Debug.evaluate("*(OS_WD*)" + pWatchdog);
    //
    // Read information from queue structure
    //
    TimeDex = Watchdog.TimeDex;
    Period  = Watchdog.Period;
    //
    // Retrieve waiting tasks
    //
    WaitingTasks = GetWaitingTasks(pWatchdog);
    //
    // Update information in the plug in window
    //
    Window.add2(WatchDogs,
                "0x" + pWatchdog.toString(16).toUpperCase(),
                GetObjectName(pWatchdog),
                (TimeDex - GlobalTime) + " (" + TimeDex + ")",
                Period,
                WaitingTasks);
  }
}

/*********************************************************************
*
*       UpdateSystemInformation()
*
*  Functions description:
*    Called from the update() function in order to retrieve watchdog
*    information and update the watchdogs list.
*
*  Parameters
*    Window: Object used to access the plug in window.
*/
function UpdateSystemInformation(Window) {
  var Status;
  var SystemTime;
  var pCurrentTask;
  var ActiveTask;
  var Libmode;
  var VersionNum;
  var Version;
  var i;

  var aFunctions = ["OS_CreateTask_XR", "OS_CreateTask_R", "OS_CreateTask_S",
                    "OS_CreateTask_SP", "OS_CreateTask_D", "OS_CreateTask_DP",
                    "OS_CreateTask_DT", "OS_CreateTask_SAFE",
                    "OS_TASK_Create_XR", "OS_TASK_Create_R", "OS_TASK_Create_S",
                    "OS_TASK_Create_SP", "OS_TASK_Create_D", "OS_TASK_Create_DP",
                    "OS_TASK_Create_DT", "OS_TASK_Create_SAFE",
                    "OS_CreateTaskEx_XR", "OS_CreateTaskEx_R", "OS_CreateTaskEx_S",
                    "OS_CreateTaskEx_SP", "OS_CreateTaskEx_D", "OS_CreateTaskEx_DP",
                    "OS_CreateTaskEx_DT", "OS_CreateTaskEx_SAFE",
                    "OS_TASK_CreateEx_XR", "OS_TASK_CreateEx_R", "OS_TASK_CreateEx_S",
                    "OS_TASK_CreateEx_SP", "OS_TASK_CreateEx_D", "OS_TASK_CreateEx_DP",
                    "OS_TASK_CreateEx_DT", "OS_TASK_CreateEx_SAFE"];

  var aLibmodes = ["Extreme Release (XR)",
                   "Release (R)",
                   "Stackcheck (S)",
                   "Stackcheck + Profiling (SP)",
                   "Debug (D)",
                   "Debug + Profiling (DP)",
                   "Debug + Trace + Profiling (DT)",
                   "Safe (SAFE)"];

  //
  // Read information
  //
  Status       = Debug.evaluate("OS_Status");
  SystemTime   = OS_Global.Time >>> 0;  // Right shift without sign extension. Used to cast from signed to unsigned.
  pCurrentTask = OS_Global.pCurrentTask;
  pActiveTask  = OS_Global.pActiveTask;
  VersionNum   = Debug.evaluate("OS_Version");
  //
  // Check if OS_Status could be evaluated
  //
  if (Status == undefined) {
    Status = "N/A";
  }
  //
  // Check Libmode
  //
  for (var i = 0; i < aFunctions.length; ++i) {
    Libmode = Debug.evaluate(aFunctions[i]);
    if (Libmode != undefined) {
      Libmode = aLibmodes[i % 8];
      break;
    } else {
      Libmode = "N/A";
    }
  }
  //
  // Compute information
  //
  if (pCurrentTask == "0") {
    pCurrentTask = "None"
  } else {
    pCurrentTask = "0x" + pCurrentTask.toString(16).toUpperCase() + " (" + GetTaskName(pCurrentTask) + ")";
  }
  if (pActiveTask == "0") {
    pActiveTask = "None"
  } else {
    pActiveTask = "0x" + pActiveTask.toString(16).toUpperCase() + " (" + GetTaskName(pActiveTask) + ")";
  }
  if (VersionNum == undefined) {
    Version = "N/A";
  } else {
    Version = Math.floor((VersionNum / 10000)) + "." + (""+(Math.floor(VersionNum / 100))).substring(1, 3);
    if ((VersionNum = VersionNum % 100) != 0) {
      Version = Version
              + (((VersionNum % 25) != 0) ? String.fromCharCode(96 + (VersionNum % 25)) : ".")
              + (((VersionNum / 25) >= 1) ? Math.floor(VersionNum / 25) : "");
    }
  }
  //
  // Update information in the plug in window
  //
  if (Status != undefined) {
    Window.add2(SystemInformation, "System Status", (Status == "OS_OK") ? "O.K." : Status);
  }
  Window.add2(SystemInformation, "System Time",   SystemTime);
  Window.add2(SystemInformation, "Current Task",  pCurrentTask);
  Window.add2(SystemInformation, "Active Task",   pActiveTask);
  
  if (Libmode != undefined) {
    Window.add2(SystemInformation, "embOS Build",   Libmode);
  }
  Window.add2(SystemInformation, "embOS Version", Version);
}

/*********************************************************************
*
*       API functions
*
**********************************************************************
*/

/*********************************************************************
*
*       getregs()
*
*  Functions description:
*    If a thread is selected in the threads window, this function returns
*    an array containing the registers r0-r12, sp, lr, pc, and psr.
*
*  Parameters
*    x: Pointer to task control block
*/
function getregs(x) {
    
  if (Debug.evaluate("SkipSaveMainContextRISCV")) {  // This is for embOS for RISCV
    var aRegs       = new Array(32);
    var Interrupted = 0;
    var PreviousPC;
    var VectoredMode;

    aRegs[riscv_sp]   = Debug.evaluate("((OS_TASK*)" + x + ")->pStack");
    aRegs[riscv_sp]  += 4;  // "pop" Counters
    aRegs[riscv_s0]   = TargetInterface.peekWord(aRegs[riscv_sp]);
    aRegs[riscv_sp]  += 4;  // "pop" s0
    aRegs[riscv_s1]   = TargetInterface.peekWord(aRegs[riscv_sp]);
    aRegs[riscv_sp]  += 4;  // "pop" s1
    aRegs[riscv_s2]   = TargetInterface.peekWord(aRegs[riscv_sp]);
    aRegs[riscv_sp]  += 4;  // "pop" s2
    aRegs[riscv_s3]   = TargetInterface.peekWord(aRegs[riscv_sp]);
    aRegs[riscv_sp]  += 4;  // "pop" s3
    aRegs[riscv_s4]   = TargetInterface.peekWord(aRegs[riscv_sp]);
    aRegs[riscv_sp]  += 4;  // "pop" s4
    aRegs[riscv_s5]   = TargetInterface.peekWord(aRegs[riscv_sp]);
    aRegs[riscv_sp]  += 4;  // "pop" s5
    aRegs[riscv_s6]   = TargetInterface.peekWord(aRegs[riscv_sp]);
    aRegs[riscv_sp]  += 4;  // "pop" s6
    aRegs[riscv_s7]   = TargetInterface.peekWord(aRegs[riscv_sp]);
    aRegs[riscv_sp]  += 4;  // "pop" s7
    aRegs[riscv_s8]   = TargetInterface.peekWord(aRegs[riscv_sp]);
    aRegs[riscv_sp]  += 4;  // "pop" s8
    aRegs[riscv_s9]   = TargetInterface.peekWord(aRegs[riscv_sp]);
    aRegs[riscv_sp]  += 4;  // "pop" s9
    aRegs[riscv_s10]  = TargetInterface.peekWord(aRegs[riscv_sp]);
    aRegs[riscv_sp]  += 4;  // "pop" s10
    aRegs[riscv_s11]  = TargetInterface.peekWord(aRegs[riscv_sp]);
    aRegs[riscv_sp]  += 4;  // "pop" s11
    aRegs[riscv_pc]   = TargetInterface.peekWord(aRegs[riscv_sp]);
    aRegs[riscv_ra]   = TargetInterface.peekWord(aRegs[riscv_sp]);
    aRegs[riscv_sp]  += 4;  // "pop" new pc / previous ra
    aRegs[riscv_sp]  += 8;  // "pop" alignment

    VectoredMode = Debug.evaluate("vtrap_entry");
    if (VectoredMode != NULL) {
      //
      // We've found vtrap_entry, thus are in vectored mode.
      //
      var OS_ISR_Local15;

      PreviousPC     = TargetInterface.peekWord(aRegs[riscv_sp] + 28);
      OS_ISR_Local15 = Debug.evaluate("OS_ISR_Local15");
      if ((PreviousPC >= VectoredMode) && (PreviousPC <= OS_ISR_Local15)) {
        Interrupted = 1;
      }
      VectoredMode = 1;
    } else {
      //
      // We haven't found vtrap_entry, thus are in direct mode.
      //
      var trap_entry;

      PreviousPC = TargetInterface.peekWord(aRegs[riscv_sp] + 60);
      trap_entry = Debug.evaluate("trap_entry");
      if ((PreviousPC >= trap_entry) && (PreviousPC <= trap_entry + 50)) {
        Interrupted = 1;
      }
      VectoredMode = 0;
    }

    if ((Interrupted == 1) && (VectoredMode == 1)){
      //
      // Task has been interrupted in vectored mode. Unstack exception frame and restore task register
      //
      aRegs[riscv_sp] += 32;  // "pop" high-level exception frame
      aRegs[riscv_ra]  = TargetInterface.peekWord(aRegs[riscv_sp]);
      aRegs[riscv_sp] += 4;  // "pop" ra
      aRegs[riscv_t0]  = TargetInterface.peekWord(aRegs[riscv_sp]);
      aRegs[riscv_sp] += 4;  // "pop" t0
      aRegs[riscv_t1]  = TargetInterface.peekWord(aRegs[riscv_sp]);
      aRegs[riscv_sp] += 4;  // "pop" t1
      aRegs[riscv_t2]  = TargetInterface.peekWord(aRegs[riscv_sp]);
      aRegs[riscv_sp] += 4;  // "pop" t2
      aRegs[riscv_a0]  = TargetInterface.peekWord(aRegs[riscv_sp]);
      aRegs[riscv_sp] += 4;  // "pop" a0
      aRegs[riscv_a1]  = TargetInterface.peekWord(aRegs[riscv_sp]);
      aRegs[riscv_sp] += 4;  // "pop" a1
      aRegs[riscv_a2]  = TargetInterface.peekWord(aRegs[riscv_sp]);
      aRegs[riscv_sp] += 4;  // "pop" a2
      aRegs[riscv_a3]  = TargetInterface.peekWord(aRegs[riscv_sp]);
      aRegs[riscv_sp] += 4;  // "pop" a3
      aRegs[riscv_a4]  = TargetInterface.peekWord(aRegs[riscv_sp]);
      aRegs[riscv_sp] += 4;  // "pop" a4
      aRegs[riscv_a5]  = TargetInterface.peekWord(aRegs[riscv_sp]);
      aRegs[riscv_sp] += 4;  // "pop" a5
      aRegs[riscv_a6]  = TargetInterface.peekWord(aRegs[riscv_sp]);
      aRegs[riscv_sp] += 4;  // "pop" a6
      aRegs[riscv_a7]  = TargetInterface.peekWord(aRegs[riscv_sp]);
      aRegs[riscv_sp] += 4;  // "pop" a7
      aRegs[riscv_t3]  = TargetInterface.peekWord(aRegs[riscv_sp]);
      aRegs[riscv_sp] += 4;  // "pop" t3
      aRegs[riscv_t4]  = TargetInterface.peekWord(aRegs[riscv_sp]);
      aRegs[riscv_sp] += 4;  // "pop" t4
      aRegs[riscv_t5]  = TargetInterface.peekWord(aRegs[riscv_sp]);
      aRegs[riscv_sp] += 4;  // "pop" t5
      aRegs[riscv_t6]  = TargetInterface.peekWord(aRegs[riscv_sp]);
      aRegs[riscv_sp] += 4;  // "pop" t6
      //
      // Since we've been interrupted, using ra would result in an erroneous stack frame. Use mepc instead!
      //
      aRegs[riscv_pc]  = TargetInterface.peekWord(aRegs[riscv_sp]);
      aRegs[riscv_sp] += 16;  // Drop remaining stack frame, so the debugger can take care of any subsequent function parameters
    } else if ((Interrupted == 1) && (VectoredMode == 0)) {
      aRegs[riscv_sp] += 32;  // "pop" high-level exception frame
      aRegs[riscv_pc]  = TargetInterface.peekWord(aRegs[riscv_sp] + 8);
      aRegs[riscv_ra]  = TargetInterface.peekWord(aRegs[riscv_sp] + 32);
      aRegs[riscv_sp] += 36;  // Drop remaining stack frame, so the debugger can take care of any subsequent function parameters
    } else {
      aRegs[riscv_ra]  = "0x00000000";  // unknown after cooperative task switch
      aRegs[riscv_t0]  = "0x00000000";  // unknown after cooperative task switch
      aRegs[riscv_t1]  = "0x00000000";  // unknown after cooperative task switch
      aRegs[riscv_t2]  = "0x00000000";  // unknown after cooperative task switch
      aRegs[riscv_a0]  = "0x00000000";  // unknown after cooperative task switch
      aRegs[riscv_a1]  = "0x00000000";  // unknown after cooperative task switch
      aRegs[riscv_a2]  = "0x00000000";  // unknown after cooperative task switch
      aRegs[riscv_a3]  = "0x00000000";  // unknown after cooperative task switch
      aRegs[riscv_a4]  = "0x00000000";  // unknown after cooperative task switch
      aRegs[riscv_a5]  = "0x00000000";  // unknown after cooperative task switch
      aRegs[riscv_a6]  = "0x00000000";  // unknown after cooperative task switch
      aRegs[riscv_a7]  = "0x00000000";  // unknown after cooperative task switch
      aRegs[riscv_t3]  = "0x00000000";  // unknown after cooperative task switch
      aRegs[riscv_t4]  = "0x00000000";  // unknown after cooperative task switch
      aRegs[riscv_t5]  = "0x00000000";  // unknown after cooperative task switch
      aRegs[riscv_t6]  = "0x00000000";  // unknown after cooperative task switch
    }
    //
    // Return default values for global pointer and thread pointer
    //
    aRegs[riscv_gp] = "0x00000000";  // unknown, this is never modified and is therefore not preserved
    aRegs[riscv_tp] = "0x00000000";  // unknown, this is never modified and is therefore not preserved

    return aRegs;
  } else {
    var aRegs       = new Array(17);
    var Interrupted = 0;
    var FPU_Used    = 0;
    var OSSwitch;
    var MPU_Used;
    var FPU_stack  = 0;

    MPU_Used = Debug.evaluate("(unsigned int*)(*(struct rt_thread *)" + x + ").MPU_Config");
    MPU_Used = (MPU_Used != NULL) ? 1 : 0;

    //
    // Retrieve current top-of-stack
    //
    aRegs[arm_sp] = Debug.evaluate("((struct rt_thread *)" + x + ")->sp");
    Interrupted = (Debug.evaluate("rt_interrupt_nest") > 0) ? 1 : 0;
    
    pCurrentTask = Debug.evaluate("&rt_current_thread->list");
    p_Current_OS_TASK = GetpTCB(pCurrentTask);
    //
    // Handle well known registers
    //
    FPU_stack       = TargetInterface.peekWord(aRegs[arm_sp]);
    aRegs[arm_sp]   += 4;  // "pop" flag
    aRegs[arm_sp]   += 4;  // "pop" psplim
    aRegs[arm_r4]    = TargetInterface.peekWord(aRegs[arm_sp]);
    aRegs[arm_sp]   += 4;  // "pop" R4
    aRegs[arm_r5]    = TargetInterface.peekWord(aRegs[arm_sp]);
    aRegs[arm_sp]   += 4;  // "pop" R5
    aRegs[arm_r6]    = TargetInterface.peekWord(aRegs[arm_sp]);
    aRegs[arm_sp]   += 4;  // "pop" R6
    aRegs[arm_r7]    = TargetInterface.peekWord(aRegs[arm_sp]);
    aRegs[arm_sp]   += 4;  // "pop" R7
    aRegs[arm_r8]    = TargetInterface.peekWord(aRegs[arm_sp]);
    aRegs[arm_sp]   += 4;  // "pop" R8
    aRegs[arm_r9]    = TargetInterface.peekWord(aRegs[arm_sp]);
    aRegs[arm_sp]   += 4;  // "pop" R9
    aRegs[arm_r10]   = TargetInterface.peekWord(aRegs[arm_sp]);
    aRegs[arm_sp]   += 4;  // "pop" R10
    aRegs[arm_r11]   = TargetInterface.peekWord(aRegs[arm_sp]);
    aRegs[arm_sp]   += 4;  // "pop" R11
    if (MPU_Used != 0) {
      //aRegs[arm_sp] += 8;  // "pop" CONTROL & Dummy
    }
    //aRegs[arm_exec]  = TargetInterface.peekWord(aRegs[arm_sp]);
    //aRegs[arm_sp]   += 4;  // "pop" exec location
    //
    // Handle remaining registers through special treatment
    //
    if (x == p_Current_OS_TASK) {  // This is for embOS for ARM
      //
      // Check if this task has been interrupted (i.e., exec location is addr. of OS_Switch() + 12)
      //
      OSSwitch = Debug.evaluate("OS_SwitchAfterISR_ARM");
      if ((aRegs[arm_exec] & ~1) == (OSSwitch + 12)) {
        Interrupted = 1;
      }
      //
      // Restore appropriate register contents
      //
      if (Interrupted == 0) {  // Remaining register contents have NOT been preserved.
        aRegs[arm_r0]  = "0x00000000";                         // unknown after cooperative task switch
        aRegs[arm_r1]  = "0x00000000";                         // unknown after cooperative task switch
        aRegs[arm_r2]  = "0x00000000";                         // unknown after cooperative task switch
        aRegs[arm_r3]  = "0x00000000";                         // unknown after cooperative task switch
        aRegs[arm_r12] = "0x00000000";                         // unknown after cooperative task switch
        aRegs[arm_lr]  = aRegs[arm_exec];                      // Set LR to exec location
        aRegs[arm_psr] = (aRegs[arm_exec] & 1) ? 0x3F : 0x1F;  // Thumb vs. ARM mode?
      } else {                 // Remaining register contents have been preserved.
        aRegs[arm_r0]  = TargetInterface.peekWord(aRegs[arm_sp]);
        aRegs[arm_sp] += 4;  // "pop" R0
        aRegs[arm_r1]  = TargetInterface.peekWord(aRegs[arm_sp]);
        aRegs[arm_sp] += 4;  // "pop" R1
        aRegs[arm_r2]  = TargetInterface.peekWord(aRegs[arm_sp]);
        aRegs[arm_sp] += 4;  // "pop" R2
        aRegs[arm_r3]  = TargetInterface.peekWord(aRegs[arm_sp]);
        aRegs[arm_sp] += 4;  // "pop" R3
        aRegs[arm_r12] = TargetInterface.peekWord(aRegs[arm_sp]);
        aRegs[arm_sp] += 4;  // "pop" R12
        aRegs[arm_lr]  = TargetInterface.peekWord(aRegs[arm_sp]);
        aRegs[arm_sp] += 4;  // "pop" LR
        aRegs[arm_pc]  = TargetInterface.peekWord(aRegs[arm_sp]);
        aRegs[arm_sp] += 4;  // "pop" PC
        aRegs[arm_psr] = TargetInterface.peekWord(aRegs[arm_sp]);
        aRegs[arm_sp] += 4;  // "pop" PSR
      }
    } 
    else 
    {                                    // This is for embOS for Cortex-M
      var OSSwitchEnd;
      var v;
      //
      // Check if this task has used the FPU
      //
      v = TargetInterface.peekWord(0xE000ED88);
      if ((v & 0x00F00000) != 0) {                    // Is the FPU enabled (CPACR b23..b20)?
        v = TargetInterface.peekWord(0xE000EF34);
        if ((v & 0xC0000000) != 0) {                  // Is the (lazy) hardware state preservation enabled (FPCCR b31..b30)?
          if ((aRegs[arm_exec] & 0x00000010) == 0) {  // Has this task used the FPU (LR b4)?
            FPU_Used = 1;
          }
        }
      }

//var PSP = TargetInterface.GetReg ("PSP");
//var MSP = TargetInterface.GetReg ("MSP");

      //
      // Check if this task has been interrupted (i.e., PC is located in OS_Switch() function)
      //
      if (FPU_stack == 0) {  // FPU registers have not been preserved, PC is located on stack 6 bytes after current SP
        aRegs[arm_pc] = TargetInterface.peekWord(aRegs[arm_sp] + (4 *  6));
      } else {              // FPU registers have been preserved, PC is located on stack 22 bytes after current SP
        aRegs[arm_pc] = TargetInterface.peekWord(aRegs[arm_sp] + (4 * 22));
      }
      OSSwitch    = Debug.evaluate("OS_Switch");
      OSSwitchEnd = Debug.evaluate("OS_Switch_End");
      if ((aRegs[arm_pc] <= OSSwitch) || (aRegs[arm_pc] >= OSSwitchEnd)) {
        Interrupted = 1;
      }
      //
      // Restore appropriate register contents
      //
      if (FPU_stack == 0) {       // FPU registers have not been preserved, use regular stack layout
          aRegs[arm_r0]   = TargetInterface.peekWord(aRegs[arm_sp]);
          aRegs[arm_sp]  += 4;  // "pop" R0
          aRegs[arm_r1]   = TargetInterface.peekWord(aRegs[arm_sp]);
          aRegs[arm_sp]  += 4;  // "pop" R1
          aRegs[arm_r2]   = TargetInterface.peekWord(aRegs[arm_sp]);
          aRegs[arm_sp]  += 4;  // "pop" R2
          aRegs[arm_r3]   = TargetInterface.peekWord(aRegs[arm_sp]);
          aRegs[arm_sp]  += 4;  // "pop" R3
          aRegs[arm_r12]  = TargetInterface.peekWord(aRegs[arm_sp]);
          aRegs[arm_sp]  += 4;  // "pop" R12
          aRegs[arm_lr]   = TargetInterface.peekWord(aRegs[arm_sp]);
          aRegs[arm_sp]  += 4;  // "pop" LR
          // aRegs[arm_pc] has already been read above
          aRegs[arm_sp]  += 4;  // "pop" PC
          aRegs[arm_psr]  = TargetInterface.peekWord(aRegs[arm_sp]);
          aRegs[arm_sp]  += 4;  // "pop" APSR
        
      } else {                   // FPU registers have been preserved, use extended stack layout

          aRegs[arm_sp]  += (4 * 16);  // "pop" S16..S31
          aRegs[arm_r0]   = TargetInterface.peekWord(aRegs[arm_sp]);
          aRegs[arm_sp]  += 4;         // "pop" R0
          aRegs[arm_r1]   = TargetInterface.peekWord(aRegs[arm_sp]);
          aRegs[arm_sp]  += 4;         // "pop" R1
          aRegs[arm_r2]   = TargetInterface.peekWord(aRegs[arm_sp]);
          aRegs[arm_sp]  += 4;         // "pop" R2
          aRegs[arm_r3]   = TargetInterface.peekWord(aRegs[arm_sp]);
          aRegs[arm_sp]  += 4;         // "pop" R3
          aRegs[arm_r12]  = TargetInterface.peekWord(aRegs[arm_sp]);
          aRegs[arm_sp]  += 4;         // "pop" R12
          aRegs[arm_lr]   = TargetInterface.peekWord(aRegs[arm_sp]);
          aRegs[arm_sp]  += 4;         // "pop" LR
          // aRegs[arm_pc] has already been read above
          aRegs[arm_sp]  += 4;         // "pop" PC
          aRegs[arm_psr]  = TargetInterface.peekWord(aRegs[arm_sp]);
          aRegs[arm_sp]  += 4;         // "pop" APSR
          aRegs[arm_sp]  += (4 * 18);  // "pop" S0..S15, FPSCR, and Res.
        
      }
    }
    return aRegs;
  }
}

/*********************************************************************
*
*       init()
*
*  Functions description:
*    This function describes the look of the threads window.
*/
function init() {
    
  Threads.clear();
  Threads.setColumns("Name", "Priority", "Status", "SP", "Stack Info", "TCB", "Time Slice", "Task Events");
  Threads.setSortByNumber("Priority");
  
  if (Threads.setColor) {
    Threads.setColor("Status", "Ready", "Executing", "Waiting");
  }
  if (0){ //Threads.setColumns2) {
    Threads.setColumns2(Timers,             "Timers",       "Name", "Hook",         "Timeout",        "Period"                                           );
    Threads.setColumns2(Mailboxes,          "Mailboxes",    "Name", "Messages",     "Message Size",   "Buffer Address", "Waiting Tasks",  "In Use"       );
    Threads.setColumns2(Queues,             "Queues",       "Name", "Messages",     "Buffer Address", "Buffer Size",    "Waiting Tasks"                  );
    Threads.setColumns2(Mutexes,            "Mutexes",      "Name", "Owner",        "Use Counter",    "Waiting Tasks"                                    );
    Threads.setColumns2(Semaphores,         "Semaphores",   "Name", "Count",        "Waiting Tasks"                                                      );
    Threads.setColumns2(MemoryPools,        "MemoryPools",  "Name", "Total Blocks", "Block Size",     "Max. Usage",     "Buffer Address", "Waiting Tasks");
    Threads.setColumns2(EventObjects,       "EventObjects", "Name", "Signaled",     "Reset Mode",     "Mask Mode",      "Waiting Tasks"                  );
    Threads.setColumns2(WatchDogs,          "WatchDogs",    "Name", "Timeout",      "Period"                                                             );
    Threads.setColumns2(SystemInformation,  SystemInformation, "Value"                                                                                   );
  }
}

/*********************************************************************
*
*       getstate()
*
*  Parameters
*    task: Task Id
*
*  Functions description:
*    This function returns the state of a task.
*/
function getState(task) {
  var WaitObj;
  var sName;
  var sText;

  //
  // Retrieve waiting object name if available
  //
  WaitObj = Debug.evaluate("(((OS_TASK*)" + task.pWaitList + ")->pWaitList)");
  sName   = GetObjectName(WaitObj);
  sText   = (WaitObj) ? (" 0x" + WaitObj.toString(16).toUpperCase()) : "";
  if (sName != "") {
    sText += " (" + sName + ")";
  }
  //
  // Return corresponding state + waiting object name if available
  //
    switch (task.stat & (RT_THREAD_STAT_MASK)) {
      case RT_THREAD_READY:
        return "Ready";
      case RT_THREAD_BLOCK:
        return "Blocked";
      case RT_THREAD_SUSPEND:
      case RT_THREAD_BLOCK:
        return "Waiting for Task Event" + sText;
      case RT_THREAD_RUNNING:
        return "Running";

      case RT_THREAD_CLOSE:
        return "Closed";
      
      default:
        return "Invalid";
    }
}

/*********************************************************************
*
*       gettls()
*
*  Functions description:
*    This function is called to retrieve the TLS address of the
*    specified task.
*/
function gettls(pTask) {
  var pTLS;

  //
  // If null is passed to gettls(pTask), get the current task.
  //
  if (pTask == null) {
    pTask = Debug.evaluate("&rt_current_thread->list");
  }
  //
  // Load the pTLS address of the selected task.
  //
  pTLS = GetpTCB(pTask);
  //
  // If the selected task does not use TLS, load tbss section address instead.
  //
  if (pTLS == 0) {
    pTLS = Debug.evaluate("__tbss_start__");
  }
  return pTLS;
}

/*********************************************************************
*
*       update()
*
*  Functions description:
*    This function is called to update the threads window and its
*    entries upon debug stop.
*/
function update() {
  var pTaskRoot;
  var pTask;
  var pCurrentTask;
  var OS_TASK;
  var p_OS_TASK;
  var Id;
  var TaskName;
  var NumActivations;
  var StackUsage;
  var TimeSliceReload;
  var TimeSliceRem;
  var sName;
  var Interrupted;

  //
  // Retrive OS_Global object
  //
  //OS_Global = Debug.evaluate("OS_Global");
  //
  // Check if embOS is existent
  //
  //if (OS_Global == undefined) {
  //  return;
  //}
  //
  // Check if embOS V4 is used and, if so, set appropiate defines
  //
  if (Debug.evaluate("OS_pRSemaRoot") != undefined) {
    CastMutex  = "(OS_RSEMA*)";
    CastSema   = "(OS_CSEMA*)";
    CastQueue  = "(OS_Q*)";
    CastMEMF   = "(OS_MEMF*)";
    RootMutex  = "OS_pRSemaRoot";
    RootSema   = "OS_pCSemaRoot";
  }
  //
  // Clear entire threads window
  //
  Threads.clear();
  //
  // Update plug in lists
  //
  if (0){//Threads.newqueue2 != undefined) {
    if (Threads.shown(Timers))             UpdateTimers(Threads);
    if (Threads.shown(Mailboxes))          UpdateMailboxes(Threads);
    if (Threads.shown(Queues))             UpdateQueues(Threads);
    if (Threads.shown(Mutexes))            UpdateMutexes(Threads);
    if (Threads.shown(Semaphores))         UpdateSemaphores(Threads);
    if (Threads.shown(MemoryPools))        UpdateMemoryPools(Threads);
    if (Threads.shown(EventObjects))       UpdateEventObjects(Threads);
    if (Threads.shown(WatchDogs))          UpdateWatchDogs(Threads);
    if (Threads.shown(SystemInformation))  UpdateSystemInformation(Threads);
  }
  //
  // Retrieve start of linked task list from target
  //
  pTaskRoot = Debug.evaluate("&(rt_object_container[0].object_list)");
  pTask     = Debug.evaluate("rt_object_container[0].object_list.next");
  
  Interrupted = (Debug.evaluate("rt_interrupt_nest") > 0) ? 1 : 0;
  
  //
  // Create new queue if at least one task is present
  //
  if (pTask != pTaskRoot) {
    if (Threads.newqueue2 == undefined) {
      Threads.newqueue("Task List");
    }
    //
    // Retrieve currently executing task, if any
    //
    pCurrentTask = Debug.evaluate("&rt_current_thread->list");
  }
  //
  // Iterate through linked list of tasks and create an entry to the queue for each
  //
  while ((pTask != pTaskRoot) && (pTask != undefined)) {
    //
    // Retrieve Task ID
    //
    Id = pTask;
    //
    // Get a pointer to this task control block
    //
    OS_TASK = GetTCB(pTask);
    p_OS_TASK = GetpTCB(pTask);
    if (OS_TASK == undefined) {
        //Threads.add(pTaskRoot.toString(16), pTask.toString(16), OS_TASK.toString(16), offset.toString(10), OS_TASK.stack_size.toString(16), "", "", "", "", undefined);
      break;
    }
    //
    // Retrieve TaskName (unavailable in some libmodes)
    //
    TaskName = GetTaskName(pTask);
    //
    // Retrieve NumActivations (unavailable in some libmodes)
    //
    NumActivations = "N/A"; //(OS_TASK.NumActivations != undefined) ? OS_TASK.NumActivations : "N/A";
    //
    // Retrieve Stackinfo (unavailable in some libmodes)
    //
    if (OS_TASK.stack_size != undefined) {
      var MaxStackUsed;
      var index;

      if (OS_TASK.stack_size > StackCheckLimit) {
        //
        // Skip stack check if stack is too big
        //
        MaxStackUsed = "??";
      } else {
        if (TargetInterface.findByte) {
          //
          // Check if TargetInterface.findByte is implemented
          //
          index = TargetInterface.findNotByte(OS_TASK.stack_addr, OS_TASK.stack_size, 0x23);
        } else {
          var abStack = TargetInterface.peekBytes(OS_TASK.stack_addr, OS_TASK.stack_size);
          for (index = 0; index < OS_TASK.stack_size; index++) {
            if (abStack[index].toString(16) != "23") {
              break;
            }
          }
        }
        MaxStackUsed = OS_TASK.stack_size - index;
      }
      StackUsage = MaxStackUsed + " / " + OS_TASK.stack_size + " @ 0x" + OS_TASK.stack_addr.toString(16).toUpperCase();
    } else {
      StackUsage = "N/A";
    }
    //
    // Retrieve TimeSliceReload and TimeSliceRem (unavailable in some libmodes)
    //
    TimeSliceReload = (OS_TASK.init_tick != undefined) ? OS_TASK.init_tick : "N/A";
    TimeSliceRem    = (OS_TASK.remaining_tick    != undefined) ? OS_TASK.remaining_tick :    "N/A";
    //
    // Add task to queue
    //
    Threads.add(TaskName,                                                                                                          // Name
                OS_TASK.current_priority.toString(10),                                                                                                  // Priority
                (pTask == pCurrentTask) ? ((0 == Interrupted) ? "Executing" : "Suspend"): getState(OS_TASK),                                                         // Status
                "0x" + OS_TASK.sp.toString(16).toUpperCase(), 
                StackUsage,                                                                                                        // Stack info
                "0x" + p_OS_TASK.toString(16).toUpperCase(),                                                                              // ID
                TimeSliceRem + " / " + TimeSliceReload,                                                                            // Time slice
                "No event",//"0x" + OS_TASK.Events.toString(16).toUpperCase(),                                                                  // Events
                //
                // If task is executing, getregs() must not be called. We therefore pass a null pointer.
                // If task is inactive, getregs() must be called. We therefore pass a pointer to this task.
                //
                ((pTask == pCurrentTask)&&(0 == Interrupted)) ? undefined : p_OS_TASK
               );
    pTask = OS_TASK.list.next;
  }
  //
  // Add IRQ to queue
  //
  Threads.add("IRQs", "", (0 == Interrupted) ? "Ready" : "Executing", "", "", "", "", "", "", undefined);
}

/*********************************************************************
*
*       getContextSwitchAddrs()
*
*  Functions description:
*    Returns an unsigned integer array containing the base addresses 
*    of all functions that complete a task switch when executed.
*/
function getContextSwitchAddrs() {
  
  var aAddrs;
  var Addr;
 
  Addr = Debug.evaluate("&rt_hw_context_switch");
  
  if (Addr == undefined) {
    Addr = Debug.evaluate("&OS_ChangeTask_STD");  
  }
  if (Addr == undefined) {
    Addr = Debug.evaluate("&OS_ChangeTask");
  }
  if (Addr == undefined) {
    return [];
  }
  aAddrs = new Array(1);
  aAddrs[0] = Addr;
  return aAddrs;
}

/*********************************************************************
*
*       getOSName()
*
*  Functions description:
*    Returns the name of the RTOS this script supplies support for
*/
function getOSName() {
  return "RtThreadRTOS";
}

/****** End Of File *************************************************/
