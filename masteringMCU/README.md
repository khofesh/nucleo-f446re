# Mastering MCU

# components

https://www.eevblog.com/forum/beginners/which-logic-analyzer-for-beginners/

# OpenOCD Debugger and Semi-hosting

Set the linker arguments

```shell
-specs=rdimon.specs -lc -lrdimon
```

Add semi-hosting run command

```
monitor arm semihosting enable
```

Add the below function call to main.c

```c
extern void initialise_monitor_handles(void);
initialise_monitor_handles();
```
