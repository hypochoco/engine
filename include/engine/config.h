//
//  config.h
//  engine
//
//  Created by Daniel Cho on 12/2/25.
//

struct Config {
    virtual ~Config() = default;
};

struct PaintConfig : public Config {
    
    static constexpr bool ACTIVE = true;
    
    static constexpr uint32_t CANVAS_WIDTH = 1024;
    static constexpr uint32_t CANVAS_HEIGHT = 1024;
        
    static inline const std::string LAYER_VERT_SHADER_PATH = "layer_vert.spv";
    static inline const std::string LAYER_FRAG_SHADER_PATH = "layer_frag.spv";
    
    static inline const std::string BRUSH_VERT_SHADER_PATH = "brush_vert.spv";
    static inline const std::string BRUSH_FRAG_SHADER_PATH = "brush_frag.spv";
    
};

struct GraphicsConfig : public Config {
    
    static constexpr uint32_t WIDTH = 800;
    static constexpr uint32_t HEIGHT = 600;

    static inline const std::string VERT_SHADER_PATH = "vert.spv";
    static inline const std::string FRAG_SHADER_PATH = "frag.spv";
        
    static constexpr int MAX_FRAMES_IN_FLIGHT = 3;
    static constexpr int NUM_TEXTURES = 4;
    static constexpr int MAX_ENTITIES = 8;
};

// more configs here

class AppConfig {
public:
    static AppConfig& instance() {
        static AppConfig inst;
        return inst;
    }

    GraphicsConfig graphicsConfig;
    PaintConfig paintConfig;

private:
    AppConfig() = default;
};
