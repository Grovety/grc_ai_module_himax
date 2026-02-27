# Validation Mask Construction

## 1. Mask syntax

Supported mask codes:

- `#` - digit (`0-9`)
- `L` - letter
- `X` - letter or digit
- `C` - any single character
- `?` - any single character (same behavior as `C`)
- `P` - one token in `<...>` format (for example `<Beijing>`)

Additional rules:

- Spaces in the mask are ignored.
- Literal characters (for example `-`) are allowed and matched as-is.
- Mask cannot contain `;`, `CR`, or `LF`.
- Max mask length in app UI: `32`.
- Device stores up to `8` masks at a time.

## 2. How to design masks

1. List real plate formats seen in your stream.
2. Create one mask per format.
3. Keep masks as strict as possible (`L/#/X` first, `C/?` only if needed).
4. For mixed-format regions (for example `singapore` and `usa`), use multiple masks.

## 3. Recognized symbols by region

### `generic`

- Digits: `0 1 2 3 4 5 6 7 8 9`
- Letters: `A B C E H K M O P T X Y`
- Special tokens: `_` (blank token, not a plate character)

Example masks:

- `X X # # # #`
- `L # # # L L`

### `china`

- Digits: `0 1 2 3 4 5 6 7 8 9`
- Letters: `A B C D E F G H I J K L M N O P Q R S T U V W X Y Z`
- Province/special tokens:
  `<Anhui> <Beijing> <Chongqing> <Fujian> <Gansu> <Guangdong> <Guangxi> <Guizhou> <Hainan> <Hebei> <Heilongjiang> <Henan> <HongKong> <Hubei> <Hunan> <InnerMongolia> <Jiangsu> <Jiangxi> <Jilin> <Liaoning> <Macau> <Ningxia> <Qinghai> <Shaanxi> <Shandong> <Shanghai> <Shanxi> <Sichuan> <Tianjin> <Tibet> <Xinjiang> <Yunnan> <Zhejiang> <police> <Gua>`
- Special tokens: `_` (blank token, not a plate character)

Use `P` for `<...>` tokens.

Example masks:

- `P L X X X X X`
- `P L # # # # #`
- `P L X X X X <police>`

### `norway`

- Digits: `0 1 2 3 4 5 6 7 8 9`
- Letters: `A B C D E F G H I J K L M N O P Q R S T U V W X Y Z Å Æ Ø`
- Special tokens: `_` (blank token, not a plate character)

Example masks:

- `L L # # # # #`
- `L L L # # # #`

### `singapore`

- Digits: `0 1 2 3 4 5 6 7 8 9`
- Letters: `A B C D E F G H I J K L M N P Q R S T U V W X Y Z`
- Special tokens: `_` (blank token, not a plate character)

Note: letter `O` is not present in the current dictionary.

Example masks:

- `L # # # # L`
- `L L # # # # L`
- `L L L # # # # L`

### `usa`

- Digits: `0 1 2 3 4 5 6 7 8 9`
- Letters: `A B C D E F G H I J K L M N O P Q R S T U V W X Y Z`
- Special tokens: `_` (blank token, not a plate character)

Example masks:

- `X X X X X X`
- `X X X X X X X`
- `X X X X X X X X`
- `L L L # # # #`
- `# # # L L L`
