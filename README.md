# NUCLEO-F446RE

https://www.st.com/en/evaluation-tools/nucleo-f446re.html

stm32cubeIDE:

https://www.st.com/en/development-tools/stm32cubeide.html

## packages

```shell
sudo dnf install stlink
```

st-info

```shell
[fahmad@ryzen ~]$  st-info --probe
Found 1 stlink programmers
  version:    V2J33S25
  serial:     066AFF505282494867173940
  flash:      524288 (pagesize: 131072)
  sram:       131072
  chipid:     0x0421
  descr:      F446
```

## videos

- https://www.youtube.com/watch?v=pHyz2-wbDw4
- https://www.youtube.com/watch?v=hyZS2p1tW-g

## other github

- https://github.com/prtzl/Embedded_videos
- https://github.com/davisjp1822/stm32_nucleo_linux

## links

- https://www.st.com/en/embedded-software/stm32cube-mcu-mpu-packages.html#tools-software

## constraint characters and constrain modifier

![constraint characters](./images/Screenshot_2022-10-22_14-39-20.png)

### implementation

`r3` has value `0x32`
![r3 0x32](./images/Screenshot_2022-10-22_14-47-08.png)

now `r0` has value `0x32`
![r0 0x32](./images/Screenshot_2022-10-22_14-47-59.png)

## X3 (crystal oscillator) is missing

![X3 missing answer](./images/Screenshot_2022-10-22_23-12-09.png)

# References

- Test-Driven Development for Embedded C (https://pragprog.com/titles/jgade/test-driven-development-for-embedded-c/)
- https://developer.arm.com/Processors/Cortex-M4 (Cortex-M4)
