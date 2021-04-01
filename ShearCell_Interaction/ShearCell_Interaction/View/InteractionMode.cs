namespace ShearCell_Interaction.View
{
    public enum InteractionMode
    {
        None,
        AddAnchor,
        MarkBoundary,
        AddInput,
        AddOutput,
        DrawInput,
        DrawOutput,

        //these are from exploration tool
        Simulate,
        AddShearCell,
        AddRigidCell,
        AddTracingPoint,
        SetTestInput,
        SetTestOutput,
        Test
    }
}