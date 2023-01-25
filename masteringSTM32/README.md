# mastering STM32 - second edition

## ignore .metadata if you already included it in git

```shell
git rm -r --cached .metadata
echo ".metadata" >> .gitignore
```

## 002-chapter05

```shell
sudo screen /dev/ttyACM0 115200
```

then, hit reset button.

to close screen session: `Ctrl+a \`

![uart](../images/Screenshot%20from%202023-01-14%2023-36-07.png)

## 005-chapter08

### uart in polling mode

<img src="../images/Screenshot%20from%202023-01-25%2022-39-29.png" alt="terminal" width="500"/>

### ringbuffer

- https://embeddedartistry.com/blog/2017/05/17/creating-a-circular-buffer-in-c-and-c/
- https://github.com/AndersKaloer/Ring-Buffer
