//光追功能暂未维护，可能无法使用。想要使用可以去找老代码。

#include <iostream>

#define REAL_TIME_SHOW

#include "main_loop.hpp"

Renderer renderer;

void recordPath() {
    mainLoopPR(renderer);
    path_recorder.smoothenPath();
    path_recorder.storgePath("PATH");
    path_recorder.showPath(renderer);
}

void generateImages() {
    path_recorder.readPath("PATH");
    renderer.init();
    path_recorder.showPath(renderer);
    path_recorder.generateImage(renderer);
}

int main() {
    //calcInitBlock({ 2, 2, 2 });
    //rayShader({ { 2, 2, 2 }, { -1.1, -1.2, -1.3 } });
    //rayShader({ { 2, 2, 2 }, { -1.1, -1.2, -1.3 } });
    //renderer.init();
    //renderer.render();
    mainLoopAA(renderer);
    // V4d a(1, 0, 0, 0), b(0, 1, 0, 0), c(0, 0, 0, 1);
    // V4d d = cross(a, b, c).normalized();
    // d.print();
    // recordPath();
    //generateImages();
    
    //getchar();
    return 0;
}