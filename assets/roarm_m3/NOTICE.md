# RoArm-M3 visualization assets

The Waveshare [roarm_description package](https://github.com/waveshareteam/roarm_ws/tree/40dbd84b553695212fab713e8465f817ba95454d/src/roarm_main/roarm_description) declares the MIT license in the included package.xml.

The Xacro is the original kinematic description. geometry.npz contains simplified triangles generated from the seven M3 visual STLs referenced in that description. The geometry is for display, not collision checking. The optional rebuild command is `python tools/bake_roarm_meshes.py <official-stl-directory> assets/roarm_m3/geometry.npz`.

MIT License

Copyright (c) the upstream roarm_description contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
