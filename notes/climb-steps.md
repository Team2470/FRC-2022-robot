# Climb Steps

## Steps
### Drive to under the MID bar, and get into postion
1.  Driven to MID bar location. Front and Back arms in starting configration
2.  Front out and backout 
    > Needs better defintion

### Automated sequence: Support Robot with front arms on MID bar
3. __FInward__: Move Front arms into to inward limit

### Automated sequence:  Support Robot with back arms on MID bar
4.  Move back arms to close to bar
5.  __B25__: Back arms to 25 degress

### Automated sequence:  Support Robot with front arms on Hight bars
6.  __FClear__: Move Front arms outward enough to clear bars
    > Need better defintion what angle is this?
7.  __B90__: Move back arms to 90 degress
8.  __FOutward__: Move Front arms outward enough to softlimit
9.  __B25__: Move back arms to 25 degress
10. __BCatch__: Move front bars to catch bar
11. __B125__: Move back arms to 125
    > Robot is supported by only the High bar

### Repeat 3 through 10 to perform traversal


## Unique actions
- B25: BackClimbAngleCommand @ 25 degress 
- B90: BackClimbAngleCommand @ 90 degress 
- B125: BackClimbAngleCommand @ 125 degress 

- FInward: FrontClimbAngleCommand @ inward limit
- FCatch: FrontClimbAngleCommand @ catch bar
  > __Question:__ Is this different from inward limit?
- FClear: FrontClimbAngleCommand @ angle to clear bar
- FOutward: FrontClimbAngleCommand @ outward limit
