#pragma once

class SynthesisConfiguration
{
public:
private:
    std::vector<std::string> m_inputScenes;
    std::vector<std::string> m_centralObjects;

    GAConfiguration m_gaConfig;
}
