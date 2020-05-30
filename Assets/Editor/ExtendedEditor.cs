/*
 * This code is part of DRL-Highway by Daniel (2020)
 * 
 */

using UnityEditor;

public class ExtendedEditor : Editor
{
    protected SerializedProperty currentProperty;

    protected SerializedProperty selectedProperty;
    protected string selectedPropertyPath;

    protected void DrawProperties(SerializedProperty prop, bool drawChildren)
    {
        string lastPropPath = string.Empty;
        foreach (SerializedProperty p in prop)
        {
            if (p.isArray && p.propertyType == SerializedPropertyType.Generic)
            {
                EditorGUILayout.BeginHorizontal();
                p.isExpanded = EditorGUILayout.Foldout(p.isExpanded, p.displayName);
                EditorGUILayout.EndHorizontal();

                if (p.isExpanded)
                {
                    EditorGUI.indentLevel++;
                    p.arraySize = EditorGUILayout.IntField("Size", p.arraySize);
                    DrawProperties(p, drawChildren);
                    EditorGUI.indentLevel--;
                }
                lastPropPath = p.propertyPath;
            }
            else
            {
                if (!string.IsNullOrEmpty(lastPropPath) && p.propertyPath.Contains(lastPropPath)) { continue; }
                lastPropPath = p.propertyPath;
                EditorGUILayout.PropertyField(p, drawChildren);
            }
        }
    }

    protected void DrawField(string propName, bool relative)
    {
        if (relative && currentProperty != null)
        {
            EditorGUILayout.PropertyField(currentProperty.FindPropertyRelative(propName), true);
        } else if (serializedObject != null)
        {
            EditorGUILayout.PropertyField(serializedObject.FindProperty(propName), true);
        }
    }
}
