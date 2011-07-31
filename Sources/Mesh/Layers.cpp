#include "Layers.h"
#include "ReportMacros.h"

Layers::Layers()
{
    isConstructed = false;
}

Layers::Layers(LayerType type, int numLev, double *lev)
{
    isConstructed = false;
    construct(type, numLev, lev);
}

Layers::~Layers()
{
}

void Layers::construct(LayerType type, int numLev, double *lev)
{
    if (isConstructed) {
        REPORT_ERROR("Layers has been constructed.")
    }

	this->type = type;
	if (type == Full) {
		this->lev.resize(numLev);
		for (int k = 0; k < this->lev.size(); ++k) {
			this->lev[k] = lev[k];
		}
		dlev.resize(numLev-1);
		for (int k = 0; k < dlev.size(); ++k) {
			dlev[k] = lev[k]-lev[k+1];
		}
	} else if (type == Half) {
        this->lev.resize(numLev-1);
        for (int k = 0; k < this->lev.size(); ++k) {
            this->lev[k] = 0.5*(lev[k]+lev[k+1]);
        }
    }

    isConstructed = true;
}
