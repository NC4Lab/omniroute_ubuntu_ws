arduino
├── arduino_scripts
│   └── MazeWallOperation
│   	└── platformio.ini
│   	└── MazeWallOperation.code-workspace
│   	└── src
│   		└── main.cpp
├── libraries
│   └── Maze_Debug
│   	└── library.json
│	└── .vscode
│   		└── c_cpp_properties.json
│   	└── src
│   		└── Maze_Debug.cpp
│   		└── Maze_Debug.h
│   └── Safe_Vector
│   	└── library.json
│	└── .vscode
│   		└── c_cpp_properties.json
│   	└── src
│   		└── Safe_Vector.cpp
│   		└── Safe_Vector.h
│   └── Cypress_Com
│   	└── library.json
│	└── .vscode
│   		└── c_cpp_properties.json
│   	└── src
│   		└── Cypress_Com.cpp
│   		└── Cypress_Com.h
│		└── Cypress_Com_Base.h
│   └── Wall_Operation
│   	└── library.json
│	└── .vscode
│   		└── c_cpp_properties.json
│   	└── src
│   		└── Wall_Operation.cpp
│   		└── Wall_Operation.h


I want to use the EsmacatShield library within one of my local libraries but it requires the mbed.h. How would I go about setting this up?

pio pkg install --library "mbed-esmacat/EsmacatShield"

''' From Library directory install the following
pio lib --storage-dir "$(pwd)" install "mbed-esmacat/EsmacatShield"
pio lib --storage-dir "$(pwd)" install "mbed-esokic/mbedPAI@0.0.0+sha.05aad811ea07"

pio lib --storage-dir "$(pwd)" install --interactive EsmacatShield
