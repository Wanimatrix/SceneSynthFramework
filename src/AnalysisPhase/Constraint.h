#ifndef CONSTRAINT_H
#define CONSTRAINT_H

class Constraint
{
public:
    Constraint() {};
    virtual ~Constraint() {};
    
    virtual bool check() = 0;
};

#endif // CONSTRAINT_H