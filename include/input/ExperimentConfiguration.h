#pragma once

class ExperimentConfiguration
{
public:
    std::string toString();
private:
    std::vector<SynthesisConfiguration> m_synthesisConfigurations;
}
