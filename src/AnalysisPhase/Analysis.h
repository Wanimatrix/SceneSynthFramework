#ifndef ANALYSIS_H
#define ANALYSIS_H

class Analysis
{
public:
    Analysis(const Input &t_input) : m_input(t_input) {};
    virtual ~Analysis() {};
    
    virtual std::vector<Constraint> run() = 0;

private:
    const Input m_input;
};

#endif // ANALYSIS_H