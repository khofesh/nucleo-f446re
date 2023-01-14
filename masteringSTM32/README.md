# mastering STM32 - second edition

## ignore .metadata if you already included it in git

```shell
git rm -r --cached .metadata
echo ".metadata" >> .gitignore
```

## 002-chapter-05

```shell
sudo screen /dev/ttyACM0 115200
```

then, hit reset button.

to close screen session: `Ctrl+a \`

![uart](../images/Screenshot%20from%202023-01-14%2023-36-07.png)
