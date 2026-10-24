function s = set_nested_struct_field(s, field_path, value)
% SET_NESTED_STRUCT_FIELD Set a nested struct field using a dynamic path.

if ischar(field_path) || isstring(field_path)
    field_path = strsplit(char(field_path), '.');
end

subs = cell(1, numel(field_path) * 2);
for i = 1:numel(field_path)
    subs{2 * i - 1} = '.';
    subs{2 * i} = field_path{i};
end

s = subsasgn(s, substruct(subs{:}), value);
end
