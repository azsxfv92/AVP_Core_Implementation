#include "CoreOptimizer.hpp"
#include <chrono>

void processData(std::unique_ptr<Frame>&& frame){
    std::cout << "Processing frame..." << frame->id << std::endl;

}

int main(){
    ObjectPool framePool;
    auto start = std::chrono::high_resolution_clock::now();
    
    for(int i = 0; i < 100; i++){
        auto frame = framePool.acquire();
        frame->id = i;

        processData(std::move(frame));

        framePool.release(std::move(frame));   
    }

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << "Total Time: " << elapsed.count() << " ms" << std::endl;
    return 0;
}