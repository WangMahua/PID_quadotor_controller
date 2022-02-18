function v_b = bound(v_o,v_upper,v_lower)
    v_b = max(v_o,v_lower);
    v_b = min(v_b,v_upper);

end