using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Documents;

namespace ShearCell_Interaction.Simulation
{
    public class SimulationInfoEventArgs : EventArgs
    {
        public List<List<Vector>> FramePosition { get; }

        public SimulationInfoEventArgs(List<List<Vector>> framePositions)
        {
            FramePosition = framePositions;
        }
    }
}