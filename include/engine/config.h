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
    
    static inline const std::string VERT_SHADER_PATH = "brush_vert.spv";
    static inline const std::string FRAG_SHADER_PATH = "brush_frag.spv";
    static inline const std::string BRUSH_PATH = "brush.png";
    
    static constexpr uint32_t CANVAS_WIDTH = 1024;
    static constexpr uint32_t CANVAS_HEIGHT = 1024;
    
    // override num_textures = 1

};

struct GraphicsConfig : public Config {
    
    static constexpr uint32_t WIDTH = 800;
    static constexpr uint32_t HEIGHT = 600;

    static inline const std::string VERT_SHADER_PATH = "vert.spv";
    static inline const std::string FRAG_SHADER_PATH = "frag.spv";
    static inline const std::vector<std::pair<std::string, std::string>> MODEL_PATHS = {
        std::make_pair("viking_room.obj", "viking_room.png")
    };
    
    static constexpr int MAX_FRAMES_IN_FLIGHT = 2;
    static constexpr int NUM_TEXTURES = 16;
    static constexpr int MAX_ENTITIES = 64;
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
