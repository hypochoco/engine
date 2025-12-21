
## High Level Goals
- Graphics engine
- Physics engine
    
---

## Notes

### Graphics API

Multi threading:
- Worker stealing
- Better cache efficiency
- Tasks have a dependency graph

### Deferred deletion

```
    // pseudocode for deferred deletion
        //struct PendingDeletion {
        //    VkBuffer buffer;
        //    VkDeviceMemory memory;
        //    VkFence fence;
        //};
        //
        //std::vector<PendingDeletion> deletions;
        //
        // // Each frame:
        //for (auto it = deletions.begin(); it != deletions.end(); ) {
        //    if (vkGetFenceStatus(device, it->fence) == VK_SUCCESS) {
        //        vkDestroyBuffer(device, it->buffer, nullptr);
        //        vkFreeMemory(device, it->memory, nullptr);
        //        it = deletions.erase(it);
        //    } else {
        //        ++it;
        //    }
        //}
```
