syms sx sy sz cx cy cz reals 
Rx = [1 0 0;0 cx sx;0 -sx cx]
Ry = [0 0 -1; 0 1 0;1 0 0]
Rz = [cz sz 0;-sz cz 0; 0 0 1]
R = Rx * Ry * Rz
