/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using UnityEngine;
using UnityEditor;

[CustomEditor(typeof(EnvironmentManager))]
public class EnvironmentManagerEditor : ExtendedEditor
{
    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        EditorGUILayout.LabelField("Environment setup", EditorStyles.boldLabel);
        DrawField("cumulativeRewardText", false);

        EditorGUILayout.Space();

        EditorGUILayout.LabelField("Highway setup", EditorStyles.boldLabel);
        DrawField("numberOfLanes", false);
        DrawField("laneWidth", false);
        serializedObject.FindProperty("laneInfo").arraySize = serializedObject.FindProperty("numberOfLanes").intValue;

        currentProperty = serializedObject.FindProperty("laneInfo");

        EditorGUILayout.BeginHorizontal("box", GUILayout.ExpandHeight(true));
        EditorGUILayout.BeginVertical(GUILayout.MaxWidth(100), GUILayout.ExpandHeight(true));
        int i = 1;
        foreach (SerializedProperty p in currentProperty)
        {
            if (GUILayout.Button("Lane " + i))
            {
                selectedPropertyPath = p.propertyPath;
            }
            i++;
        }
        EditorGUILayout.EndVertical();
        EditorGUILayout.Separator();
        EditorGUILayout.BeginVertical(GUILayout.ExpandHeight(true));
        if (!string.IsNullOrEmpty(selectedPropertyPath))
        {
            selectedProperty = serializedObject.FindProperty(selectedPropertyPath);
            if (selectedProperty != null)
            {
                DrawProperties(selectedProperty, true);
            }
            else
            {
                EditorGUILayout.LabelField("Select a lane");
            }
        }
        EditorGUILayout.EndVertical();
        EditorGUILayout.EndHorizontal();

        serializedObject.ApplyModifiedProperties();
        

        //DrawDefaultInspector();
    }
}
